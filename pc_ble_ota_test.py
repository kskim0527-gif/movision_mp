import asyncio
import os
import time
from bleak import BleakScanner, BleakClient

# --- 설정 사항 ---
DEVICE_NAME = "Mando HUD T" # ESP32에서 사용하는 블루투스 기기 이름
OTA_SERVICE_UUID = "80323655-3537-410b-ab8c-cc07fc9673ce"
OTA_CTRL_UUID    = "80323656-3537-410b-ab8c-cc07fc9673ce" # OTA 상태 제어용
OTA_DATA_UUID    = "80323657-3537-410b-ab8c-cc07fc9673ce" # 펌웨어 데이터 전송용

# 업데이트 할 바이너리 파일 경로 (디폴트)
BIN_FILE_PATH = r"update\consumer\movision_ota.bin"

# 한 번에 전송할 바이트 수 (MTU 247을 사용하는 기기에 맞춰 여유있게 200~240 설정)
CHUNK_SIZE = 240 

async def run(file_path):
    if not os.path.exists(file_path):
        print(f"에러: 펌웨어 파일({file_path})을 찾을 수 없습니다.")
        print("먼저 Vscode 환경에서 'Build'를 완료하여 파일을 생성해주세요.")
        return

    print(f"'{DEVICE_NAME}' BLE 기기를 찾는 중입니다...")
    devices = await BleakScanner.discover(timeout=5.0)
    target_device = None
    
    for d in devices:
        if d.name and DEVICE_NAME in d.name:
            target_device = d
            break

    if not target_device:
        print(f"기기 '{DEVICE_NAME}' 를 찾을 수 없습니다.")
        print("ESP32 기기가 켜져 있고, 화면에서 'OTA Update' 대기 모드로 진입했는지 확인해주세요.")
        return

    print(f"기기 발견! {target_device.name} ({target_device.address}). 연결 중...")
    
    # BLE 기기와 연결 시작
    async with BleakClient(target_device.address) as client:
        if not client.is_connected:
            print("기기 연결에 실패했습니다.")
            return

        print("연결 완료!")

        # 1. 기기 통신 상태 확인 (가장 먼저 시작 명령 보내기)
        print("\n[1/3] 업데이트 시작 명령(BEGIN OTA)을 전송합니다...")
        try:
            # 값 0x01(Begin)을 Write(Response=True)로 보냄
            await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x01]), response=True)
            # ESP32 내부의 파티션 Erase 작업이 완료될 때까지 잠시 대기
            await asyncio.sleep(1) 
        except Exception as e:
            print(f"명령 전송 실패: {e}. (기기가 OTA 모드가 아닐 수 있습니다)")
            return

        # 2. PC에서 파일 읽고 전송
        file_size = os.path.getsize(file_path)
        print(f"\n[2/3] 펌웨어 전송을 시작합니다! (총 {file_size} 바이트)")
        
        start_time = time.time()
        with open(file_path, "rb") as f:
            bytes_sent = 0
            while True:
                chunk = f.read(CHUNK_SIZE)
                if not chunk:
                    break
                
                # Write Without Response (No Response) 방식으로 빠르게 펌웨어 데이터 밀어넣기
                await client.write_gatt_char(OTA_DATA_UUID, chunk, response=False)
                bytes_sent += len(chunk)

                # 퍼센트 진행률 표시
                percent = (bytes_sent / file_size) * 100
                print(f"\r진행률: {percent:.1f}% ({bytes_sent}/{file_size} 바이트 전송됨)", end="")

        print()
        elapsed_time = time.time() - start_time
        speed = (file_size / elapsed_time) / 1024
        print(f"전송 완료! 소요 시간: {elapsed_time:.1f}초 (평균 속도: {speed:.1f} KB/s)")

        # 3. 종료 및 재부팅 명령 전송
        print("\n[3/3] 업데이트 처리 완료 명령(END OTA)을 전송합니다...")
        await client.write_gatt_char(OTA_CTRL_UUID, bytearray([0x02]), response=True)
        print("\n성공했습니다! 기기가 곧 자동으로 재부팅되며 새로운 펌웨어로 동작합니다. 🎉")
        
        # 기기가 통신을 끊기 전까지 약간 대기
        await asyncio.sleep(2)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="PC 기반 ESP32 BLE 예약 OTA 툴")
    parser.add_argument("--file", type=str, default=BIN_FILE_PATH, help="전송할 펌웨어 .bin 파일의 경로")
    args = parser.parse_args()
    
    # 비동기 시작
    asyncio.run(run(args.file))
