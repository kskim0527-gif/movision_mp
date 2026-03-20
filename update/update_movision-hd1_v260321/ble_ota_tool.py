"""
movision BLE OTA 업데이트 클라이언트
=====================================
이 스크립트는 ble_update.bat에서 자동 실행됩니다.
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

# ── BLE UUID ──
UUID_SVC  = "80323655-3537-410b-ab8c-cc07fc9673ce"
UUID_CTRL = "80323656-3537-410b-ab8c-cc07fc9673ce"
UUID_DATA = "80323657-3537-410b-ab8c-cc07fc9673ce"

CMD_BEGIN_APP     = 1
CMD_END_APP       = 2
CMD_BEGIN_STORAGE = 3
CMD_END_STORAGE   = 4
CMD_SYNC          = 5

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
if os.path.exists(os.path.join(SCRIPT_DIR, "movision.bin")):
    BUILD_DIR = SCRIPT_DIR
else:
    BUILD_DIR = os.path.join(SCRIPT_DIR, "..", "build")

APP_BIN     = os.path.join(BUILD_DIR, "movision.bin")
STORAGE_BIN = os.path.join(BUILD_DIR, "storage.bin")

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
        print("  ❌ 주변에 BLE 장치가 없습니다.")
        return None
    print(f"\n  {'No':>3}  {'장치 이름':<28}  {'주소':<20}  RSSI")
    print("  " + "-" * 60)
    for i, (dev, adv) in enumerate(all_entries):
        tag  = "★" if (dev, adv) in priority else " "
        rssi = adv.rssi if adv.rssi is not None else "?"
        print(f"  {i+1:>3}  {tag}{(dev.name or '(이름없음)'):<27}  {dev.address:<20}  {rssi}")
    print()
    while True:
        choice = input(f"  장치 번호 선택 (1~{len(all_entries)}, 0=새로고침): ").strip()
        if choice == "0": return await scan_and_select()
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(all_entries): return all_entries[idx][0].address
        except ValueError: pass

async def send_file(client, data, cmd_begin, cmd_end, label, chunk_size):
    total = len(data)
    begin_cmd = bytes([cmd_begin]) + struct.pack("<I", total)
    print(f"\n[{label}] 시작 ({total//1024} KB, 청크={chunk_size}B)")
    await client.write_gatt_char(UUID_CTRL, begin_cmd, response=True)
    erase_wait = 4.0 if cmd_begin == CMD_BEGIN_APP else 12.0
    print(f"  파티션 지우는 중... ({erase_wait:.0f}초 대기)", flush=True)
    await asyncio.sleep(erase_wait)
    sent, pkt_cnt = 0, 0
    t_start = time.time()
    while sent < total:
        chunk = data[sent : sent + chunk_size]
        await client.write_gatt_char(UUID_DATA, chunk, response=False)
        sent += len(chunk); pkt_cnt += 1
        progress_bar(sent, total, label)
        if pkt_cnt % 100 == 0:
            await client.write_gatt_char(UUID_CTRL, bytes([CMD_SYNC]), response=True)
        elif pkt_cnt % 10 == 0:
            await asyncio.sleep(0)
    elapsed = time.time() - t_start
    print(f"\n  ✅ 완료! {elapsed:.1f}초, 평균 {(total/1024)/elapsed:.0f} KB/s")
    await client.write_gatt_char(UUID_CTRL, bytes([cmd_end]), response=True)

async def run_update(address, do_app, do_storage):
    print(f"\n[2/3] 연결 중: {address}")
    try:
        async with BleakClient(address, timeout=20.0) as client:
            print(f"  ✅ 연결 성공! MTU={client.mtu_size}")
            try: await client.request_mtu(512)
            except: pass
            await asyncio.sleep(1.5)
            chunk_size = max(20, client.mtu_size - 3)
            print("\n[3/3] 파일 전송 시작")
            if do_app and do_storage:
                with open(APP_BIN, "rb") as f: app_data = f.read()
                with open(STORAGE_BIN, "rb") as f: storage_data = f.read()
                # 펌웨어
                await send_file(client, app_data, CMD_BEGIN_APP, CMD_SYNC, "펌웨어", chunk_size)
                # Storage
                await send_file(client, storage_data, CMD_BEGIN_STORAGE, CMD_END_STORAGE, "Storage", chunk_size)
            elif do_app:
                with open(APP_BIN, "rb") as f: await send_file(client, f.read(), CMD_BEGIN_APP, CMD_END_APP, "펌웨어", chunk_size)
            elif do_storage:
                with open(STORAGE_BIN, "rb") as f: await send_file(client, f.read(), CMD_BEGIN_STORAGE, CMD_END_STORAGE, "Storage", chunk_size)
            print("\n  🎉 업데이트 완료! 3초 후 재시작됩니다.")
            await asyncio.sleep(4.0)
            return True
    except Exception as e:
        print(f"\n  ❌ 오류: {e}")
    return False

def check_files():
    has_app, has_sto = os.path.exists(APP_BIN), os.path.exists(STORAGE_BIN)
    print("\n파일 확인:")
    print(f"  {'✅' if has_app else '❌'} 펌웨어  : movision.bin")
    print(f"  {'✅' if has_sto else '❌'} Storage : storage.bin")
    if not has_app and not has_sto: return False, False
    if has_app and has_sto:
        c = input("\n선택 (1:모두, 2:펌웨어만, 3:Storage만): ").strip()
        return (c in ('1','2')), (c in ('1','3'))
    return has_app, has_sto

def main():
    clr(); banner()
    app, sto = check_files()
    if not app and not sto: return
    addr = asyncio.run(scan_and_select())
    if addr: asyncio.run(run_update(addr, app, sto))
    input("\n종료하려면 엔터...")

if __name__ == "__main__": main()
