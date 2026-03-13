import asyncio
import os
import time
from bleak import BleakScanner, BleakClient

# --- 설정 사항 ---
DEVICE_NAME = "MOVISION HUD1"
OTA_SERVICE_UUID = "80323655-3537-410b-ab8c-cc07fc9673ce"
OTA_CTRL_UUID    = "80323656-3537-410b-ab8c-cc07fc9673ce"
OTA_DATA_UUID    = "80323657-3537-410b-ab8c-cc07fc9673ce"

CHUNK_SIZE = 244 
SYNC_INTERVAL = 128 * 1024 

async def run(file_path):
    if not os.path.exists(file_path):
        print(f"에러: 파일을 찾을 수 없습니다: {file_path}")
        return

    print(f"'{DEVICE_NAME}' 찾는 중...")
    target_device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name and DEVICE_NAME in d.name, timeout=10.0
    )

    if not target_device:
        print(f"기기 '{DEVICE_NAME}'를 찾을 수 없습니다.")
        return

    print(f"연결 중: {target_device.address}")
    async with BleakClient(target_device, timeout=20.0) as client:
        print(f"연결 완료! (매칭된 MTU: {client.mtu_size})")

        file_size = os.path.getsize(file_path)
        print(f"\n[1/3] STORAGE 업데이트 시작 (크기: {file_size} 바이트)...")
        size_bytes = file_size.to_bytes(4, byteorder='little')
        
        # 1. 시작 명령 전송
        await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x03]) + size_bytes, response=True)
        
        print(f"기기 지우기 대기 중 (약 30초)... 기기 로그를 확인하세요.")
        await asyncio.sleep(30.0) 
        print("지우기 완료 예상. 데이터 전송을 시작합니다.")

        # 2. 데이터 전송
        print(f"[2/3] 데이터 전송 (Chunk: {CHUNK_SIZE}, Sync: {SYNC_INTERVAL//1024}KB)")
        start_time = time.time()
        last_log_time = start_time
        bytes_sent = 0
        last_sync = 0
        
        with open(file_path, "rb") as f:
            while bytes_sent < file_size:
                chunk = f.read(CHUNK_SIZE)
                if not chunk: break
                
                await client.write_gatt_char(OTA_DATA_UUID, chunk, response=False)
                bytes_sent += len(chunk)
                
                if bytes_sent - last_sync >= SYNC_INTERVAL:
                    print(f"\n  >> Sync Point ({bytes_sent//1024}KB)...", end="", flush=True)
                    await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x05]), response=True)
                    print(" [OK]")
                    last_sync = bytes_sent
                
                curr_t = time.time()
                if curr_t - last_log_time >= 0.5 or bytes_sent == file_size:
                    elapsed = curr_t - start_time
                    speed = (bytes_sent / 1024) / elapsed if elapsed > 0 else 0
                    percent = (bytes_sent / file_size) * 100
                    print(f"\r  진행률: {percent:5.1f}% ({bytes_sent//1024:4d}/{file_size//1024:4d} KB) | 속도: {speed:5.1f} KB/s", end="", flush=True)
                    last_log_time = curr_t
                    
        total_time = time.time() - start_time
        print(f"\n  전송 완료! 평균 속도: {(file_size/1024)/total_time:.1f} KB/s")

        # 3. 종료 명령 전송
        print("\n[3/3] 업데이트 완료 명령 전송...")
        await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x04]), response=True)
        print("\n모든 작업이 성공적으로 끝났습니다! 장치가 3초 후 재부팅됩니다. 🎉")

if __name__ == "__main__":
    import sys
    f_path = "storage.bin"
    if len(sys.argv) > 1:
        f_path = sys.argv[1]
    asyncio.run(run(f_path))
