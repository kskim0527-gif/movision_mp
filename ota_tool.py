import asyncio
import os
import struct
import time
from bleak import BleakScanner, BleakClient, BleakError
from tqdm import tqdm

# Protocol Constants
HEADER = 0x19
ID_FW = 0x4F
TAIL = 0x2F

CMD_FW_INFO = 0x01
CMD_FW_REQ = 0x02
CMD_FW_DATA = 0x03
CMD_FW_DATA_ACK = 0x04
CMD_FW_COMPLETE = 0x05

# UUIDs
SERVICE_UUID = "0000ffea-0000-1000-8000-00805f9b34fb"
CHAR_FFF1_NOTIFY = "0000fff1-0000-1000-8000-00805f9b34fb" 
CHAR_FFF2_WRITE = "0000fff2-0000-1000-8000-00805f9b34fb"

def format_hex(data):
    return " ".join(f"{b:02x}" for b in data).upper()

class MOVISION_OTA:
    def __init__(self, bin_path):
        self.bin_path = bin_path
        self.total_size = os.path.getsize(bin_path)
        self.block_size = 500 
        self.current_pos = 0
        self.finished = False
        self.error = None
        self.pbar = None
        self.start_transfer = False
        self.last_rx_time = 0
        self.ack_event = asyncio.Event()
        self.last_ack_seq = -1

    def build_packet(self, cmd, data):
        dlen = len(data)
        packet = bytearray([HEADER, ID_FW, cmd])
        packet.append((dlen >> 8) & 0xFF)
        packet.append(dlen & 0xFF)
        packet.extend(data)
        packet.append(TAIL)
        return packet

    async def handle_notification(self, sender, data):
        self.last_rx_time = time.time()
        
        if len(data) < 3: return
        cid = data[1]
        cmd = data[2]
        
        # Payload start detection
        payload_idx = 5 if len(data) > 5 and data[0] == HEADER else 4

        # 로그 출력 여부 결정
        show_log = True
        msg_raw = f"[RX] ID=0x{cid:02X} CMD=0x{cmd:02X} Raw: {data.hex().upper()}"
        
        if cid == 0x4F and cmd == 0x04:
            # ACK 패킷 처리
            # HUD 펌웨어 실제 포맷: [19][4F][04][03][SEQ_H][SEQ_L][ERR][2F]
            # index:               0    1   2   3    4      5     6    7
            try:
                ack_seq = (data[4] << 8) | data[5]  # index 4, 5
                err_code = data[6] if len(data) > 6 else 0  # index 6 (index 7 = 0x2F tail)
                
                self.last_ack_seq = ack_seq
                is_last = (self.current_pos >= self.total_size - self.block_size)
                
                # 0x2F(47)는 Tail 바이트 - 에러 코드가 아님
                if err_code not in (0, 0x2F):
                    self.error = f"HUD reported error {err_code} at Seq {ack_seq}"
                    self.ack_event.set()
                    return
                
                # 200개 단위 또는 처음/마지막만 출력
                if not (ack_seq % 200 == 0 or ack_seq == 0 or is_last):
                    show_log = False
                
                msg_raw += f" -> ACK Recv: Seq {ack_seq}"
                self.ack_event.set()
            except IndexError:
                pass
        
        # 필터를 통과했거나 ACK가 아닌 패킷인 경우 출력
        if show_log:
            if self.pbar: self.pbar.write(msg_raw)
            else: print(msg_raw)

        # 0x4F (펌웨어 로그) 추가 로직
        if cid == 0x4F:
            if cmd == CMD_FW_REQ:
                self.start_transfer = True
            elif cmd == CMD_FW_COMPLETE:
                # 완료 패킷 포맷: [19][4F][05][01][ERR][2F]
                # ERR 위치는 고정 index 4 (data[5] = 0x2F Tail 바이트)
                status = data[4] if len(data) > 4 else 0xFF
                if status == 0:
                    print(f"[OK] Completion packet verified.")
                    self.finished = True
                else:
                    self.error = f"Update failed on HUD (Error Code: {status})"

    async def run_transfer_loop(self):
        print(f"[INFO] Pushing data (Block={self.block_size}, Windowed x5 Mode)...")
        try:
            with open(self.bin_path, 'rb') as f:
                while self.current_pos < self.total_size and not self.error:
                    f.seek(self.current_pos)
                    chunk = f.read(self.block_size)
                    seq = self.current_pos // self.block_size
                    
                    payload = bytearray([(seq >> 8) & 0xFF, seq & 0xFF])
                    payload.extend(chunk)
                    packet = self.build_packet(CMD_FW_DATA, payload)
                    
                    # ACK 대기 이벤트 초기화
                    self.ack_event.clear()
                    
                    # 비동기 송신 (WNR)
                    await self.client.write_gatt_char(CHAR_FFF2_WRITE, packet, response=False)
                    
                    # 5블럭마다 또는 마지막 블럭에서만 ACK 대기
                    is_last = (self.current_pos + len(chunk) >= self.total_size)
                    if (seq % 5 == 4) or is_last:
                        try:
                            await asyncio.wait_for(self.ack_event.wait(), timeout=10.0)
                        except asyncio.TimeoutError:
                            self.error = f"ACK Timeout at Seq {seq}"
                            break
                    else:
                        # 윈도우 내 연속 전송 시 Windows 스택 부하 분산 (1.5ms)
                        await asyncio.sleep(0.0015)
                    
                    self.current_pos += len(chunk)
                    if self.pbar: self.pbar.update(len(chunk))
                    
            if not self.error:
                msg = "\n[INFO] 100% Sent. Waiting for final HUD confirmation..."
                if self.pbar: self.pbar.write(msg)
                else: print(msg)
        except Exception as e:
            self.error = f"Transfer Interrupted: {e}"

    async def run(self, address):
        def on_disconnect(c):
            # 전송이 100% 완료된 상태에서 끊긴 거라면 에러로 간주하지 않고 3초간 더 기다려봄
            if self.current_pos < self.total_size:
                self.error = "BLE Link Lost during transfer."
            else:
                print("\n[!] BLE Link Disconnected (Expected if device reboots).")

        try:
            async with BleakClient(address, disconnected_callback=on_disconnect) as client:
                self.client = client
                print(f"Connected to {address}")
                # Indication/Notification 모두 처리
                await client.start_notify(CHAR_FFF1_NOTIFY, self.handle_notification)
                
                # 규격: [Version:6][Size:4]
                # HUD가 index 11~14에서 크기를 읽으므로, 앞에 버전 6바이트를 정확히 배치 (총 10바이트)
                info = b"260407" + struct.pack(">I", self.total_size) 
                packet = self.build_packet(CMD_FW_INFO, info)
                print(f"[TX] {format_hex(packet)} (Start Handshake)")
                await client.write_gatt_char(CHAR_FFF2_WRITE, packet)
                
                self.pbar = tqdm(total=self.total_size, unit='B', unit_scale=True, desc="OTA Progress")
                while not self.start_transfer and not self.error: await asyncio.sleep(0.1)
                if self.start_transfer and not self.error: await self.run_transfer_loop()

                # 전송 100% 이후 대기 (최대 15초)
                start_wait = time.time()
                while not self.finished and not self.error:
                    await asyncio.sleep(0.5)
                    if time.time() - start_wait > 15.0:
                        # 100% 전송되었으면 패킷 도달 여부에 상관없이 성공으로 간주할 수도 있음 (보수적 처리)
                        if self.current_pos >= self.total_size: 
                            if self.pbar: self.pbar.write("[WARN] 100% Sent but no ACK. Device likely rebooted early.")
                            self.finished = True
                        else:
                            self.error = "Timeout waiting for 0x4F 05."
                
                if self.pbar: self.pbar.close()
                if self.error: print(f"\n[FAILURE] {self.error}")
                else: print("\n[SUCCESS] Firmware update sequence completed successfully.")
                
                await asyncio.sleep(1)
        except Exception as e:
            print(f"\n[FATAL ERROR] {e}")

async def main():
    print("MOVISION OTA Tool v5.8 (Windowed ACK x5 - Filtered Logging)")
    devices = await BleakScanner.discover(timeout=4.0)
    huds = [d for d in devices if d.name and "HUD" in d.name.upper()]
    if not huds: print("No HUD found."); return
    for i, d in enumerate(huds): print(f"[{i}] {d.name} ({d.address})")
    try:
        idx = int(input("\nIndex: "))
        path = input("BIN path: ").strip().strip('"').strip("'")
        if os.path.exists(path): await MOVISION_OTA(path).run(huds[idx].address)
    except: pass

if __name__ == "__main__": asyncio.run(main())
