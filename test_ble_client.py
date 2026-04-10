#!/usr/bin/env python3
"""
ESP32 BLE HUD 테스트 클라이언트
PC에서 ESP32의 BLE 서비스에 연결하여 명령을 전송하고 응답을 받습니다.
"""

import asyncio
from bleak import BleakClient, BleakScanner
import struct

# ESP32 BLE 서비스 및 특성 UUID
HUD_SERVICE_UUID = "0000ffea-0000-1000-8000-00805f9b34fb"
HUD_CHAR_READ_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"  # FFF1 (Read/Notify)
HUD_CHAR_WRITE_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"  # FFF2 (Write)

# 프로토콜 상수
PROTOCOL_HEADER = 0x19
PROTOCOL_ID = 0x4D
PROTOCOL_TAIL = 0x2F

# 디바이스 이름
DEVICE_NAME = "MOVISION HUD1"

# 알림 콜백
def notification_handler(sender, data):
    """FFF1 채널에서 받은 알림 데이터 처리"""
    print(f"\n[ESP32->PC] FFF1 알림 수신 ({len(data)} bytes):")
    print(f"  Hex: {' '.join(f'{b:02X}' for b in data)}")
    if len(data) >= 5:
        header = data[0]
        id_byte = data[1]
        cmd = data[2]
        dlen = data[3]
        print(f"  Header: 0x{header:02X}, ID: 0x{id_byte:02X}, CMD: 0x{cmd:02X}, DataLen: {dlen}")

async def scan_and_connect():
    """BLE 디바이스 스캔 및 연결"""
    print(f"BLE 디바이스 스캔 중... (찾는 디바이스: {DEVICE_NAME})")
    
    devices = await BleakScanner.discover(timeout=10.0)
    
    target_device = None
    for device in devices:
        if device.name and DEVICE_NAME in device.name:
            target_device = device
            print(f"\n디바이스 발견: {device.name}")
            print(f"  주소: {device.address}")
            break
    
    if target_device is None:
        print(f"\n오류: '{DEVICE_NAME}' 디바이스를 찾을 수 없습니다.")
        print("사용 가능한 디바이스:")
        for device in devices:
            if device.name:
                print(f"  - {device.name} ({device.address})")
        return None
    
    return target_device

def create_command(cmd, data_bytes):
    """HUD 명령 패킷 생성"""
    # 형식: [0x19 0x4D cmd data_length data... 0x2F]
    packet = bytearray([PROTOCOL_HEADER, PROTOCOL_ID, cmd, len(data_bytes)])
    packet.extend(data_bytes)
    packet.append(PROTOCOL_TAIL)
    return bytes(packet)

async def main():
    """메인 함수"""
    print("=" * 60)
    print("ESP32 BLE HUD 테스트 클라이언트")
    print("=" * 60)
    
    # 디바이스 스캔 및 연결
    device = await scan_and_connect()
    if device is None:
        return
    
    try:
        async with BleakClient(device.address) as client:
            print(f"\n연결 성공: {device.name}")
            print(f"MTU: {client.mtu_size}")
            
            # FFF1 알림 활성화 (start_notify가 자동으로 CCCD를 설정함)
            print("\nFFF1 알림 활성화 중...")
            await client.start_notify(HUD_CHAR_READ_UUID, notification_handler)
            print("FFF1 알림 활성화 완료")
            
            print("\n" + "=" * 60)
            print("명령 전송 준비 완료")
            print("=" * 60)
            print("\n사용 가능한 테스트 명령:")
            print("  1. GPS 상태 확인 (cmd=0x04)")
            print("     예: 19 4D 04 01 00 2F (GPS 수신 불가)")
            print("     예: 19 4D 04 01 01 2F (GPS 수신 가능)")
            print("  2. TBT 방향 표시 (cmd=0x01)")
            print("     예: 19 4D 01 05 00 00 00 00 00 2F (arrow_0.png)")
            print("  3. Speed Mark (cmd=0x03)")
            print("     예: 19 4D 03 02 00 00 2F")
            print("  4. Safety Drive (cmd=0x02)")
            print("     예: 19 4D 02 06 01 32 01 01 23 45 2F")
            print("\n명령을 입력하세요 (hex, 공백 구분, 예: 19 4D 04 01 00 2F)")
            print("또는 'q'를 입력하여 종료")
            print("-" * 60)
            
            while True:
                try:
                    user_input = input("\n명령 입력: ").strip()
                    
                    if user_input.lower() == 'q':
                        break
                    
                    # Hex 문자열을 바이트로 변환
                    hex_bytes = user_input.split()
                    try:
                        data = bytes([int(b, 16) for b in hex_bytes])
                    except ValueError as e:
                        print(f"오류: 잘못된 hex 값 - {e}")
                        continue
                    
                    if len(data) < 5:
                        print("오류: 명령이 너무 짧습니다 (최소 5 bytes 필요)")
                        continue
                    
                    # FFF2 채널로 명령 전송
                    print(f"\n[PC->ESP32] FFF2 명령 전송 ({len(data)} bytes):")
                    print(f"  Hex: {' '.join(f'{b:02X}' for b in data)}")
                    
                    await client.write_gatt_char(HUD_CHAR_WRITE_UUID, data, response=True)
                    print("  전송 완료")
                    
                    # 응답 대기 (짧은 딜레이)
                    await asyncio.sleep(0.5)
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"오류 발생: {e}")
            
            # 알림 비활성화
            await client.stop_notify(HUD_CHAR_READ_UUID)
            print("\n연결 종료")
            
    except Exception as e:
        print(f"\n연결 오류: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n프로그램 종료")

