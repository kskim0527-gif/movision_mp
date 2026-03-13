"""
movision BLE OTA 업데이트 클라이언트
=====================================
이 스크립트는 BLE_업데이트.bat에서 자동 실행됩니다.
"""

import asyncio
import os
import sys
import time
import struct
import traceback

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("[오류] bleak 라이브러리가 없습니다.")
    print("       pip install bleak  명령으로 설치하세요.")
    input("\n아무 키나 누르면 종료...")
    sys.exit(1)

# ── BLE UUID (ota_ble.c 와 동일, little-endian → standard UUID) ──
# ota_service_uuid128 = {CE,73,96,FC,07,CC,8C,AB,0B,41,37,35,55,36,32,80}
UUID_SVC  = "80323655-3537-410b-ab8c-cc07fc9673ce"
UUID_CTRL = "80323656-3537-410b-ab8c-cc07fc9673ce"
UUID_DATA = "80323657-3537-410b-ab8c-cc07fc9673ce"

CMD_BEGIN_APP     = 1
CMD_END_APP       = 2
CMD_BEGIN_STORAGE = 3
CMD_END_STORAGE   = 4
CMD_SYNC          = 5

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
# 현재 폴더에서 업데이트 파일을 찾도록 변경
APP_BIN     = os.path.join(SCRIPT_DIR, "movision.bin")
# movision.bin이 없으면 movision_factory.bin이 있는지 확인
if not os.path.exists(APP_BIN) and os.path.exists(os.path.join(SCRIPT_DIR, "movision_factory.bin")):
    APP_BIN = os.path.join(SCRIPT_DIR, "movision_factory.bin")
STORAGE_BIN = os.path.join(SCRIPT_DIR, "storage.bin")


def clr():
    os.system("cls")

def banner():
    print("=" * 60)
    print("   movision BLE OTA 업데이트 도구")
    print("=" * 60)

def progress_bar(current, total, label="전송"):
    percent = int(current * 100 / total) if total > 0 else 0
    filled  = percent // 2
    bar     = "█" * filled + "░" * (50 - filled)
    kb_c    = current // 1024
    kb_t    = total // 1024
    print(f"\r  [{bar}] {percent:3d}%  {kb_c:4d}/{kb_t:4d} KB  {label}", end="", flush=True)


async def scan_and_select():
    """BLE 장치 스캔 후 사용자가 선택 (bleak 2.0 호환)"""
    print("\n[1/3] BLE 장치 스캔 중... (8초)")
    raw = await BleakScanner.discover(timeout=8.0, return_adv=True)

    entries = list(raw.values())

    def get_rssi(entry):
        _, adv = entry
        return adv.rssi if adv.rssi is not None else -999

    entries.sort(key=get_rssi, reverse=True)

    priority, others = [], []
    for entry in entries:
        dev, adv = entry
        name = (dev.name or "").lower()
        if any(k in name for k in ["movision", "movition", "hud", "esp", "mando"]):
            priority.append(entry)
        else:
            others.append(entry)

    all_entries = priority + others
    if not all_entries:
        print("  ❌ 주변에 BLE 장치가 없습니다. PC 블루투스를 확인하세요.")
        return None

    print(f"\n  {'No':>3}  {'장치 이름':<28}  {'주소':<20}  RSSI")
    print("  " + "-" * 60)
    for i, (dev, adv) in enumerate(all_entries):
        tag  = "★" if (dev, adv) in priority else " "
        rssi = adv.rssi if adv.rssi is not None else "?"
        print(f"  {i+1:>3}  {tag}{(dev.name or '(이름없음)'):<27}  {dev.address:<20}  {rssi}")

    print()
    while True:
        choice = input(f"  연결할 장치 번호 (1~{len(all_entries)}, 0=새로고침): ").strip()
        if choice == "0":
            return await scan_and_select()
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(all_entries):
                return all_entries[idx][0].address
        except ValueError:
            pass
        print("  잘못된 입력입니다.")


async def send_file(client, data, cmd_begin, cmd_end, label, chunk_size):
    """파일 데이터를 BLE로 고속 청크 전송"""
    total     = len(data)
    begin_cmd = bytes([cmd_begin]) + struct.pack("<I", total)

    print(f"\n[{label}] 시작 ({total//1024} KB, 청크={chunk_size}B)")
    await client.write_gatt_char(UUID_CTRL, begin_cmd, response=True)

    # ── 파티션 지우기 대기 (고정값) ──────────────────────────────
    # ESP32 플래시 지우기: 4KB 블록당 ~10ms → 4MB = ~10초 (이상 발생 가능)
    # 실제 환경(Storage 4MB)에서 지우기가 20초 이상 걸리는 경우가 있어 대기 시간을 넉넉히 상향
    erase_wait = 10.0 if cmd_begin == CMD_BEGIN_APP else 25.0
    print(f"  파티션 지우는 중... ({erase_wait:.0f}초 대기)", flush=True)
    await asyncio.sleep(erase_wait)

    sent       = 0
    pkt_cnt    = 0
    t_start    = time.time()

    # SYNC 주기: 50개 패킷마다 (흐름 제어 강화)
    SYNC_EVERY = 50

    while sent < total:
        chunk = data[sent : sent + chunk_size]
        await client.write_gatt_char(UUID_DATA, chunk, response=False)
        sent    += len(chunk)
        pkt_cnt += 1
        progress_bar(sent, total, label)

        # 매 SYNC_EVERY 패킷마다 흐름 제어
        if pkt_cnt % SYNC_EVERY == 0:
            await client.write_gatt_char(UUID_CTRL, bytes([CMD_SYNC]), response=True)
        # BLE 버퍼 overflow 방지 (10패킷마다 이벤트 루프 양보)
        elif pkt_cnt % 10 == 0:
            await asyncio.sleep(0)

    elapsed = time.time() - t_start
    speed   = (total / 1024) / elapsed if elapsed > 0 else 0
    print(f"\n  ✅ 완료! {elapsed:.1f}초, 평균 {speed:.0f} KB/s")

    await client.write_gatt_char(UUID_CTRL, bytes([cmd_end]), response=True)
    await asyncio.sleep(0.5)


async def run_update(address, do_app, do_storage):
    print(f"\n[2/3] 연결 중: {address}")
    print("  (연결에 최대 20초 소요될 수 있습니다...)")

    for attempt in range(1, 4):
        try:
            async with BleakClient(address, timeout=20.0) as client:
                print(f"  ✅ 연결 성공!  기본 MTU={client.mtu_size}")

                # MTU 협상 요청
                try:
                    await client.request_mtu(512)
                    print(f"  MTU 협상 완료: {client.mtu_size}")
                except Exception:
                    pass

                # 연결 안정화 대기
                await asyncio.sleep(1.5)

                # 동적 청크 크기 (MTU - 3바이트 헤더)
                chunk_size = max(20, client.mtu_size - 3)
                print(f"  청크 크기: {chunk_size} 바이트")

                # OTA 서비스 확인
                svc_uuids = [str(s.uuid).lower() for s in client.services]
                found = any(UUID_SVC in u for u in svc_uuids)
                if not found:
                    print(f"\n  ⚠️  OTA 서비스를 찾지 못했습니다.")
                    print(f"     찾는 UUID : {UUID_SVC}")
                    print(f"     장치 서비스:")
                    for u in svc_uuids:
                        print(f"       {u}")
                    return False

                print("  ✅ OTA 서비스 확인!")
                print("\n[3/3] 파일 전송 시작")

                if do_app and do_storage:
                    # ── 펌웨어 + Storage 동시 업데이트 ─────────────────
                    # 순서: CMD=1 → 앱데이터 → CMD=3 → Storage데이터 → CMD=4
                    # CMD=2(END_APP)는 3초 후 재시작을 유발하므로 사용하지 않음!
                    # CMD=3(BEGIN_STORAGE)이 자동으로 esp_ota_end() 호출함
                    with open(APP_BIN, "rb") as f:
                        app_data = f.read()
                    with open(STORAGE_BIN, "rb") as f:
                        storage_data = f.read()

                    # 1) 펌웨어 전송 시작 (CMD=1)
                    total_app = len(app_data)
                    begin_app = bytes([CMD_BEGIN_APP]) + struct.pack("<I", total_app)
                    print(f"\n[펌웨어] 시작 ({total_app//1024} KB, 청크={chunk_size}B)")
                    await client.write_gatt_char(UUID_CTRL, begin_app, response=True)
                    print("  파티션 지우는 중... (10초 대기)", flush=True)
                    await asyncio.sleep(10.0)

                    # 2) 펌웨어 데이터 전송
                    sent, pkt = 0, 0
                    t0 = time.time()
                    while sent < total_app:
                        chunk = app_data[sent:sent + chunk_size]
                        await client.write_gatt_char(UUID_DATA, chunk, response=False)
                        sent += len(chunk); pkt += 1
                        progress_bar(sent, total_app, "펌웨어")
                        if pkt % 50 == 0:
                            await client.write_gatt_char(UUID_CTRL, bytes([CMD_SYNC]), response=True)
                        elif pkt % 10 == 0:
                            await asyncio.sleep(0)
                    print(f"\n  ✅ 완료! {time.time()-t0:.1f}초, {total_app//1024/(time.time()-t0):.0f} KB/s")

                    # 3) CMD=3(BEGIN_STORAGE) → ESP32가 펌웨어 확정 + Storage 시작
                    total_sto = len(storage_data)
                    begin_sto = bytes([CMD_BEGIN_STORAGE]) + struct.pack("<I", total_sto)
                    print(f"\n[Storage] 시작 ({total_sto//1024} KB, 청크={chunk_size}B)")
                    await client.write_gatt_char(UUID_CTRL, begin_sto, response=True)
                    print("  파티션 지우는 중... (25초 대기)", flush=True)
                    await asyncio.sleep(25.0)

                    # 4) Storage 데이터 전송
                    sent, pkt = 0, 0
                    t0 = time.time()
                    while sent < total_sto:
                        chunk = storage_data[sent:sent + chunk_size]
                        await client.write_gatt_char(UUID_DATA, chunk, response=False)
                        sent += len(chunk); pkt += 1
                        progress_bar(sent, total_sto, "Storage")
                        if pkt % 50 == 0:
                            await client.write_gatt_char(UUID_CTRL, bytes([CMD_SYNC]), response=True)
                        elif pkt % 10 == 0:
                            await asyncio.sleep(0)
                    print(f"\n  ✅ 완료! {time.time()-t0:.1f}초, {total_sto//1024/(time.time()-t0):.0f} KB/s")

                    # 5) CMD=4(END_STORAGE) → 재시작
                    await client.write_gatt_char(UUID_CTRL, bytes([CMD_END_STORAGE]), response=True)

                elif do_app:
                    # 펌웨어만
                    with open(APP_BIN, "rb") as f:
                        await send_file(client, f.read(),
                                        CMD_BEGIN_APP, CMD_END_APP,
                                        "펌웨어", chunk_size)

                elif do_storage:
                    # Storage만
                    with open(STORAGE_BIN, "rb") as f:
                        await send_file(client, f.read(),
                                        CMD_BEGIN_STORAGE, CMD_END_STORAGE,
                                        "Storage", chunk_size)

                print("\n" + "=" * 60)
                print("  🎉 업데이트 완료!")
                print("     장치가 3초 후 자동 재시작됩니다.")
                print("=" * 60)
                await asyncio.sleep(4.0)
                return True

        except Exception as e:
            err_msg = str(e) or repr(e)
            print(f"\n  ❌ 연결 오류 (시도 {attempt}/3): {err_msg}")
            if not err_msg:
                traceback.print_exc()
            if attempt < 3:
                print("  3초 후 재시도...")
                await asyncio.sleep(3)
            else:
                print("\n  ── 연결 실패 원인 확인 ──")
                print("  1. 장치가 이미 다른 기기와 BLE 연결되어 있지 않은지 확인")
                print("  2. 장치를 재부팅 후 다시 시도")
                print("  3. Windows 블루투스 설정에서 장치를 '제거' 후 재시도")
    return False


def check_files():
    """업데이트할 파일 선택"""
    has_app     = os.path.exists(APP_BIN)
    has_storage = os.path.exists(STORAGE_BIN)

    print("\n업데이트 파일 확인:")
    if has_app:
        app_filename = os.path.basename(APP_BIN)
        print(f"  ✅ 펌웨어  ({app_filename}) : {os.path.getsize(APP_BIN)//1024} KB")
    else:
        print(f"  ❌ 펌웨어  (movision.bin) : 없음")

    if has_storage:
        print(f"  ✅ Storage (storage.bin)  : {os.path.getsize(STORAGE_BIN)//1024} KB")
    else:
        print(f"  ❌ Storage (storage.bin)  : 없음")

    if not has_app and not has_storage:
        print("\n  ⚠️  업데이트할 파일이 없습니다. 먼저 ESP-IDF 빌드를 실행하세요.")
        return False, False

    print()
    if has_app and has_storage:
        print("  업데이트 선택:")
        print("  1. 펌웨어 + Storage 모두")
        print("  2. 펌웨어만")
        print("  3. Storage만")
        c = input("  선택 (1/2/3): ").strip()
        return (c in ("1", "2")), (c in ("1", "3"))

    return has_app, has_storage


def main():
    clr()
    banner()

    do_app, do_storage = check_files()
    if not do_app and not do_storage:
        input("\n아무 키나 누르면 종료...")
        return

    address = asyncio.run(scan_and_select())
    if not address:
        input("\n아무 키나 누르면 종료...")
        return

    asyncio.run(run_update(address, do_app, do_storage))
    input("\n아무 키나 누르면 종료...")


if __name__ == "__main__":
    main()
