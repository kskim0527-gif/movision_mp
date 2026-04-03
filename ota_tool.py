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
        hex_data = format_hex(data)
        
        # 무조건 로깅 (전송 중에 너무 많은 로그 방지를 위해 100% 이후거나 시작 전일 때만 상세 출력)
        if not self.start_transfer or self.current_pos >= self.total_size:
            print(f"\n[RX] {hex_data}")

        if len(data) < 5: return
        cmd = data[2]
        
        # Payload start detection
        payload_idx = 4
        if data[0] == HEADER:
            d_1 = data[3]
            d_2 = (data[3] << 8) | data[4]
            # If d_1 is length and tail is at 3+1+d_1
            if 3+1+d_1 < len(data) and data[3+1+d_1] == TAIL: payload_idx = 4
            elif 3+2+d_2 < len(data) and data[3+2+d_2] == TAIL: payload_idx = 5

        if cmd == CMD_FW_REQ:
            result = data[payload_idx]
            sbs = (data[payload_idx+1] << 8) | data[payload_idx+2]
            if result == 1:
                self.block_size = sbs
                self.start_transfer = True
            else:
                self.error = "HUD rejected update (Busy or Size Error)."
        
        elif cmd == CMD_FW_COMPLETE:
            status = data[payload_idx]
            if status == 0:
                print(f"[OK] Completion packet verified.")
                self.finished = True
            else:
                self.error = f"Update failed on HUD (Error Code: {status})"

    async def run_transfer_loop(self):
        print(f"[INFO] Pushing data (Block={self.block_size}, Delay=20ms)...")
        try:
            with open(self.bin_path, 'rb') as f:
                while self.current_pos < self.total_size and not self.error:
                    f.seek(self.current_pos)
                    chunk = f.read(self.block_size)
                    seq = self.current_pos // self.block_size
                    payload = bytearray([(seq >> 8) & 0xFF, seq & 0xFF])
                    payload.extend(chunk)
                    packet = self.build_packet(CMD_FW_DATA, payload)
                    
                    # WNR (Write No Response)
                    await self.client.write_gatt_char(CHAR_FFF2_WRITE, packet, response=False)
                    
                    self.current_pos += len(chunk)
                    if self.pbar: self.pbar.update(len(chunk))
                    
                    await asyncio.sleep(0.02)
                    
            print("\n[INFO] 100% Sent. Waiting for HUD state change...")
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
                
                info = bytearray([6]) + b"1.10.x" + struct.pack(">I", self.total_size)
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
                            print("[WARN] 100% Sent but no ACK. Device likely rebooted early.")
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
    print("MOVISION OTA Tool v3.7 (Indication Support)")
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
