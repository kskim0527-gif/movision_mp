import asyncio
import os
import time
import argparse
import struct
from bleak import BleakScanner, BleakClient

# --- 설정 사항 ---
DEVICE_NAME = "MOVISION HUD1"
OTA_SERVICE_UUID = "80323655-3537-410b-ab8c-cc07fc9673ce"
OTA_CTRL_UUID    = "80323656-3537-410b-ab8c-cc07fc9673ce"
OTA_DATA_UUID    = "80323657-3537-410b-ab8c-cc07fc9673ce"

CHUNK_SIZE = 244 
SYNC_INTERVAL = 128 * 1024 

async def send_data(client, data, label="Data"):
    total_size = len(data)
    start_time = time.time()
    last_log_time = start_time
    bytes_sent = 0
    last_sync = 0
    
    while bytes_sent < total_size:
        chunk = data[bytes_sent : bytes_sent + CHUNK_SIZE]
        await client.write_gatt_char(OTA_DATA_UUID, chunk, response=False)
        bytes_sent += len(chunk)
        
        if bytes_sent - last_sync >= SYNC_INTERVAL:
            print(f"\n  >> {label} Sync Point ({bytes_sent//1024}KB)...", end="", flush=True)
            await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x05]), response=True)
            print(" [OK]")
            last_sync = bytes_sent
            
        curr_t = time.time()
        if curr_t - last_log_time >= 0.5 or bytes_sent == total_size:
            elapsed = curr_t - start_time
            speed = (bytes_sent / 1024) / elapsed if elapsed > 0 else 0
            percent = (bytes_sent / total_size) * 100
            print(f"\r  {label} 진행률: {percent:5.1f}% ({bytes_sent//1024:4d}/{total_size//1024:4d} KB) | 속도: {speed:5.1f} KB/s", end="", flush=True)
            last_log_time = curr_t
    
    total_t = time.time() - start_time
    print(f"\n  {label} 완료! 시간: {total_t:.1f}초, 평균 속도: {(total_size/1024)/total_t:.1f} KB/s")

async def run(file_path):
    if not os.path.exists(file_path):
        print(f"에러: 파일을 찾을 수 없습니다: {file_path}")
        return

    with open(file_path, "rb") as f:
        header = f.read(16)
        magic, app_size, storage_size, reserved = struct.unpack("<4sIII", header)
        if magic != b"UPGD":
            print("에러: 유효한 통합 패키지가 아닙니다.")
            return
        app_data = f.read(app_size)
        storage_data = f.read(storage_size)

    print(f"통합 패키지: App({app_size//1024}KB), Storage({storage_size//1024}KB)")
    print(f"'{DEVICE_NAME}' 찾는 중...")
    target_device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name and DEVICE_NAME in d.name, timeout=10.0
    )

    if not target_device: return
    
    async with BleakClient(target_device, timeout=20.0) as client:
        print(f"연결 성공! (MTU: {client.mtu_size})")

        # 1. Firmware
        if app_size > 0:
            print(f"\n[1/2] Firmware 업데이트 시작...")
            await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x01]) + app_size.to_bytes(4, 'little'), response=True)
            await asyncio.sleep(0.5)
            await send_data(client, app_data, "Firmware")

        # 2. Storage
        if storage_size > 0:
            print(f"\n[2/2] Storage 업데이트 시작...")
            await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x03]) + storage_size.to_bytes(4, 'little'), response=True)
            print(f"  기기 지우기 완료 대기 (약 30초)...")
            await asyncio.sleep(30.0)
            await send_data(client, storage_data, "Storage")

        # 3. Finish
        print("\n[완료] 재부팅 명령 전송...")
        await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x04]), response=True)
        print("모든 작업 성공! 장치가 3초 후 재부팅됩니다. 🎉")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", default="update.bin")
    args = parser.parse_args()
    asyncio.run(run(args.file))
