"""
movision BLE OTA 업데이트 스크립트
====================================
사용법:
  1. 펌웨어만 업데이트:
     python ble_ota.py --app build/movision.bin

  2. Storage(이미지/GIF)만 업데이트:
     python ble_ota.py --storage build/storage.bin

  3. 펌웨어 + Storage 동시 업데이트:
     python ble_ota.py --app build/movision.bin --storage build/storage.bin

  4. BLE 장치 검색만:
     python ble_ota.py --scan

설치 필요:
  pip install bleak
"""

import asyncio
import argparse
import os
import sys
import time
from bleak import BleakClient, BleakScanner

# ── BLE UUID (ota_ble.c 와 동일) ─────────────────────────────
OTA_SVC_UUID  = "803236550000-410B-AB8C-CC07FC9673CE"
OTA_CTRL_UUID = "803236560000-410B-AB8C-CC07FC9673CE"
OTA_DATA_UUID = "803236570000-410B-AB8C-CC07FC9673CE"

# UUID를 표준 형식으로 변환 (리틀엔디안 바이트 배열 → 표준 UUID 문자열)
# ota_service_uuid128 = {0xCE,0x73,0x96,0xFC,0x07,0xCC,0x8C,0xAB,0x0B,0x41,0x37,0x35,0x55,0x36,0x32,0x80}
# 리틀엔디안이므로 역순으로 읽음
def bytes_to_uuid(b):
    return f"{b[15]:02x}{b[14]:02x}{b[13]:02x}{b[12]:02x}-" \
           f"{b[11]:02x}{b[10]:02x}-" \
           f"{b[9]:02x}{b[8]:02x}-" \
           f"{b[7]:02x}{b[6]:02x}-" \
           f"{b[5]:02x}{b[4]:02x}{b[3]:02x}{b[2]:02x}{b[1]:02x}{b[0]:02x}"

SVC_BYTES  = [0xCE,0x73,0x96,0xFC,0x07,0xCC,0x8C,0xAB,0x0B,0x41,0x37,0x35,0x55,0x36,0x32,0x80]
CTRL_BYTES = [0xCE,0x73,0x96,0xFC,0x07,0xCC,0x8C,0xAB,0x0B,0x41,0x37,0x35,0x56,0x36,0x32,0x80]
DATA_BYTES = [0xCE,0x73,0x96,0xFC,0x07,0xCC,0x8C,0xAB,0x0B,0x41,0x37,0x35,0x57,0x36,0x32,0x80]

UUID_SVC  = bytes_to_uuid(SVC_BYTES)
UUID_CTRL = bytes_to_uuid(CTRL_BYTES)
UUID_DATA = bytes_to_uuid(DATA_BYTES)

CHUNK_SIZE = 244   # BLE MTU 한계 (ota_ble.c 기준)

CMD_BEGIN_APP     = bytes([1])
CMD_END_APP       = bytes([2])
CMD_BEGIN_STORAGE = bytes([3])
CMD_END_STORAGE   = bytes([4])
CMD_SYNC          = bytes([5])


def progress_bar(current, total, prefix=""):
    percent = current * 100 // total if total > 0 else 0
    bar = "█" * (percent // 2) + "░" * (50 - percent // 2)
    kb_cur = current // 1024
    kb_tot = total // 1024
    print(f"\r{prefix} [{bar}] {percent:3d}% ({kb_cur}/{kb_tot} KB)", end="", flush=True)


async def scan_devices():
    """BLE 장치 스캔"""
    print("BLE 장치 스캔 중... (5초)")
    devices = await BleakScanner.discover(timeout=5.0)
    print(f"\n발견된 장치 {len(devices)}개:")
    for d in devices:
        print(f"  [{d.address}]  {d.name or '(이름없음)'}  RSSI={d.rssi}")
    return devices


async def find_movision():
    """movision 장치 자동 검색"""
    print("movision 장치 검색 중...")
    devices = await BleakScanner.discover(timeout=8.0)
    for d in devices:
        name = (d.name or "").lower()
        if "movision" in name or "movition" in name or "hud" in name:
            print(f"  ✅ 발견: [{d.address}] {d.name}")
            return d.address
    
    # 이름으로 못 찾으면 OTA 서비스 UUID로 검색
    print("  이름으로 못 찾음. UUID로 재검색...")
    for d in devices:
        print(f"  확인 중: {d.address} ({d.name})")
    
    print("\n장치를 찾지 못했습니다.")
    print("장치 주소를 직접 입력하세요 (예: AA:BB:CC:DD:EE:FF):")
    return input("> ").strip()


async def send_file(client, data, cmd_begin, cmd_end, label):
    """파일 데이터를 BLE로 전송"""
    total = len(data)
    size_bytes = total.to_bytes(4, 'little')
    
    # BEGIN 명령 전송 (cmd + 4바이트 크기)
    begin_cmd = bytes([cmd_begin[0]]) + size_bytes
    print(f"\n[{label}] 시작 명령 전송 ({total//1024} KB) ...")
    await client.write_gatt_char(UUID_CTRL, begin_cmd, response=True)
    await asyncio.sleep(1.0)   # 지우기 대기
    
    # 데이터 청크 전송
    sent = 0
    start_time = time.time()
    while sent < total:
        chunk = data[sent:sent + CHUNK_SIZE]
        await client.write_gatt_char(UUID_DATA, chunk, response=False)
        sent += len(chunk)
        progress_bar(sent, total, label)
        
        # 매 8KB마다 동기화
        if sent % (8 * 1024) == 0:
            await client.write_gatt_char(UUID_CTRL, CMD_SYNC, response=True)
    
    elapsed = time.time() - start_time
    speed = (total / 1024) / elapsed if elapsed > 0 else 0
    print(f"\n  완료! {elapsed:.1f}초, {speed:.1f} KB/s")
    
    # END 명령
    await client.write_gatt_char(UUID_CTRL, cmd_end, response=True)
    await asyncio.sleep(0.5)


async def run_ota(address, app_path=None, storage_path=None):
    print(f"\n연결 중: {address}")
    async with BleakClient(address, timeout=15.0) as client:
        print(f"연결 성공! MTU={client.mtu_size}")
        
        # 서비스 확인
        svcs = [str(s.uuid).lower() for s in client.services]
        if UUID_SVC.lower() not in svcs:
            print(f"⚠️  OTA 서비스를 찾지 못했습니다.")
            print(f"   장치 서비스: {svcs}")
            return False
        
        print("✅ OTA 서비스 확인!")
        
        if app_path:
            with open(app_path, 'rb') as f:
                app_data = f.read()
            await send_file(client, app_data, CMD_BEGIN_APP, CMD_END_APP, "펌웨어")
        
        if storage_path:
            with open(storage_path, 'rb') as f:
                storage_data = f.read()
            await send_file(client, storage_data,
                          CMD_BEGIN_STORAGE, CMD_END_STORAGE, "Storage")
        
        print("\n🎉 업데이트 완료! 장치가 3초 후 재시작됩니다...")
        return True


def main():
    parser = argparse.ArgumentParser(description="movision BLE OTA 업데이트")
    parser.add_argument("--scan",    action="store_true", help="BLE 장치 스캔")
    parser.add_argument("--addr",    type=str, help="장치 BLE 주소 (자동검색 생략)")
    parser.add_argument("--app",     type=str, help="펌웨어 .bin 파일 경로")
    parser.add_argument("--storage", type=str, help="storage.bin 파일 경로")
    args = parser.parse_args()
    
    if args.scan:
        asyncio.run(scan_devices())
        return
    
    if not args.app and not args.storage:
        parser.print_help()
        print("\n예시:")
        print("  python ble_ota.py --storage build/storage.bin")
        print("  python ble_ota.py --app build/movision.bin --storage build/storage.bin")
        return
    
    # 파일 존재 확인
    for path, name in [(args.app, "펌웨어"), (args.storage, "Storage")]:
        if path and not os.path.exists(path):
            print(f"❌ {name} 파일 없음: {path}")
            sys.exit(1)
        if path:
            print(f"  {name}: {path} ({os.path.getsize(path)//1024} KB)")
    
    # 주소 확인
    address = args.addr
    if not address:
        address = asyncio.run(find_movision())
    
    if not address:
        print("❌ 장치 주소를 알 수 없습니다.")
        sys.exit(1)
    
    # OTA 실행
    success = asyncio.run(run_ota(address, args.app, args.storage))
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
