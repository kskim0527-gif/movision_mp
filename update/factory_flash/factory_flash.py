import os
import sys
import csv
import datetime
import argparse

import serial.tools.list_ports
import esptool
import esp_idf_nvs_partition_gen.nvs_partition_gen as nvs_gen

# ==========================================
# CONFIGURATION
# ==========================================
if getattr(sys, 'frozen', False):
    SCRIPT_DIR = os.path.dirname(sys.executable)
else:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

CSV_FILE = os.path.join(SCRIPT_DIR, 'serial.csv')
NVS_BIN_FILE = os.path.join(SCRIPT_DIR, 'nvs_generated.bin')
NVS_CSV_FILE = os.path.join(SCRIPT_DIR, 'nvs_temp.csv')

BIN_MAP = {
    'bootloader.bin': '0x0',
    'partition-table.bin': '0x8000',
    'nvs_generated.bin': '0x9000',
    'ota_data_initial.bin': '0xF000',
    'movision.bin': '0x20000',
    'storage.bin': '0x3B2000'
}

def ensure_csv_exists():
    if not os.path.exists(CSV_FILE):
        print(f"\n[NOTICE] {CSV_FILE} not found. Generating a sample file.")
        with open(CSV_FILE, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['SERIAL_NUM', 'APP_REG_NUM', 'USED_TIME'])
            writer.writerow(['DEV001', 'ABC01DEF', ''])
            writer.writerow(['DEV002', 'XYZ98765', ''])
        print(f"-> Please fill in 'serial.csv' with the required serial numbers and run again.")
        sys.exit(0)

def get_next_unused_serial():
    rows = []
    selected_index = -1
    
    with open(CSV_FILE, 'r', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        rows.append(header)
        
        for i, row in enumerate(reader):
            while len(row) < 3:
                row.append('')
            
            used_time = row[2].strip()
            if not used_time and selected_index == -1:
                selected_index = i + 1 
            
            rows.append(row)
            
    if selected_index == -1:
        print("\n[COMPLETE] All serial numbers in the CSV file have already been used!")
        sys.exit(0)
        
    return rows, selected_index

def mark_as_used(rows, index):
    rows[index][2] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(CSV_FILE, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerows(rows)

def generate_nvs_csv(serial_num, app_reg_num):
    with open(NVS_CSV_FILE, 'w', newline='', encoding='utf-8') as f:
        f.write("key,type,encoding,value\n")
        f.write("storage,namespace,,\n")
        f.write(f"serial_num,data,string,{serial_num}\n")
        f.write(f"app_reg_num,data,string,{app_reg_num}\n")

def generate_nvs_bin():
    print(f" -> Generating NVS binary... ({NVS_CSV_FILE})")
    old_argv = sys.argv
    sys.argv = ['nvs_partition_gen.py', 'generate', NVS_CSV_FILE, NVS_BIN_FILE, '0x6000']
    try:
        nvs_gen.main()
        return True
    except SystemExit as e:
        if e.code != 0:
            print("\n[ERROR] NVS binary generation failed.")
            return False
        return True
    except Exception as e:
        print(f"\n[ERROR] Exception during NVS generation: {e}")
        return False
    finally:
        sys.argv = old_argv

def flash_all_files(port):
    flash_args = []
    print("\n[VALIDATING DEPENDENCIES]")
    for filename, offset in BIN_MAP.items():
        file_path = os.path.join(SCRIPT_DIR, filename)
        if os.path.exists(file_path):
            flash_args.extend([offset, file_path])
            print(f" - {filename:<25} -> {offset}")
        else:
            print(f" - [SKIPPED] {filename:<13} -> File not found.")
    
    if not flash_args:
        print("\n[ERROR] No firmware files found to flash!")
        return False

    print(f"\n-> Flashing all files to PORT: {port} at baudrate: 921600...")
    cmd_args = ['--port', port, '--baud', '921600', 'write_flash'] + flash_args
    
    try:
        esptool.main(cmd_args)
        return True
    except SystemExit as e:
        if e.code != 0:
            print(f"\n[ERROR] esptool flashing failed (Exit code: {e.code})")
            return False
        return True
    except Exception as e:
        print(f"\n[ERROR] Exception during esptool flashing: {e}")
        return False

def select_com_port(default_port):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("\n[WARNING] No COM ports detected! Please check your USB connection.")
        return default_port

    print("\n--- Available COM Ports ---")
    for i, port in enumerate(ports):
        print(f" [{i+1}] {port.device} - {port.description}")
    print("---------------------------")
    
    while True:
        selection = input("\n> Select a port by number (1, 2...) or type the port name (e.g. COM4) [Press Enter for Auto-detect]: ").strip()
        
        if not selection:
            # Auto-detect logic: try to find a port with 'USB' or 'CH340' or 'CP210x'
            for port in ports:
                if 'USB' in port.description or 'UART' in port.description:
                    return port.device
            return ports[0].device # return first available if none matched explicitly
            
        if selection.isdigit():
            idx = int(selection) - 1
            if 0 <= idx < len(ports):
                return ports[idx].device
            else:
                print("[ERROR] Invalid number. Try again.")
        else:
            # User typed string manually
            if not selection.upper().startswith('COM'):
                selection = f"COM{selection}"
            return selection.upper()

def get_best_current_port(last_port):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
        
    # 우선적으로 이전에 썼던 포트가 동일하게 살아있다면 그대로 사용
    for p in ports:
        if p.device == last_port:
            return last_port
            
    # 기존 포트가 사라졌다면, 새로 연결된 USB/UART 포트를 찾아 자동 할당
    for p in ports:
        if 'USB' in p.description or 'UART' in p.description or 'CH340' in p.description:
            return p.device
            
    # 정 못 찾겠으면 인식된 첫 번째 포트 반환
    return ports[0].device

def main():
    parser = argparse.ArgumentParser(description="Movition Factory Standalone Multi-Flash Tool")
    parser.add_argument('-p', '--port', default=None, help='COM Port')
    args = parser.parse_args()

    ensure_csv_exists()

    print("\n==============================================")
    print("   Movition Factory Standalone Flash Tool     ")
    print("==============================================")

    # Port Selection Logic
    active_port = args.port
    if not active_port:
        active_port = select_com_port(default_port='COM4')
        
    print("\nWorking Directory:", SCRIPT_DIR)
    print(f"Active Port: {active_port}")

    while True:
        rows, target_idx = get_next_unused_serial()
        target_serial = rows[target_idx][0]
        target_reg = rows[target_idx][1]

        print(f"\n==============================================")
        print(f" [WAITING FOR DEVICE]")
        print(f" Target Serial Number : '{target_serial}'")
        print(f" Target App Reg Number: '{target_reg}'")
        print(f"==============================================")
        
        cmd = input("\n> Connect the USB cable to the next device and press [Enter] (or type 'q' to quit): ")
        if cmd.lower() == 'q':
            break

        # 동적 포트 감지 로직 (기기 교체 시 COM 포트가 바뀌는 현상 완벽 방어)
        new_port = get_best_current_port(active_port)
        if not new_port:
            print("\n[ERROR] No COM port detected at all! Check USB connection.")
            continue
            
        if new_port != active_port:
            print(f"\n[AUTO-DETECT] Port changed automatically: {active_port} -> {new_port}")
            active_port = new_port

        # 1. Generate NVS
        generate_nvs_csv(target_serial, target_reg)
        if not generate_nvs_bin():
            continue
        
        # 2. Flash ESP
        if flash_all_files(active_port):
            # 3. Mark as used in CSV
            mark_as_used(rows, target_idx)
            print(f"\n[SUCCESS] '{target_serial}' successfully flashed and injected! (Row {target_idx+1} in serial.csv is marked as used)")
            print(f"----------------------------------------------")
        else:
            print(f"\n[FAILED] Flash error occurred. Please check the cable connection on {active_port} and try again.")

if __name__ == '__main__':
    # Multiprocessing support for PyInstaller
    import multiprocessing
    multiprocessing.freeze_support()
    main()
