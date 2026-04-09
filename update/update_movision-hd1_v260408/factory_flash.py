import os
import sys
import csv
import subprocess
import datetime
import argparse

# ==========================================
# 기본 설정
# ==========================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, 'serial.csv')
NVS_BIN_FILE = os.path.join(SCRIPT_DIR, 'nvs_generated.bin')
NVS_CSV_FILE = os.path.join(SCRIPT_DIR, 'nvs_temp.csv')

# 각 바이너리 파일과 해당 오프셋 정의 (update_tool.html 내 동일하게 매핑)
BIN_MAP = {
    'bootloader.bin': '0x0',
    'partition-table.bin': '0x8000',
    'nvs_generated.bin': '0x9000',      # 스크립트가 자동 생성하는 고유 NVS
    'ota_data_initial.bin': '0xF000',
    'movision.bin': '0x20000',
    'storage.bin': '0x3B2000'
}

# ESP-IDF 툴 경로 찾기
IDF_PATH = os.environ.get('IDF_PATH')
if not IDF_PATH:
    IDF_PATH = r'C:\esp'

NVS_GEN_TOOL = os.path.join(IDF_PATH, 'components', 'nvs_flash', 'nvs_partition_generator', 'nvs_partition_gen.py')
ESPTOOL = 'esptool.py'

def check_environment():
    if not os.path.exists(NVS_GEN_TOOL):
        print(f"\n[오류] NVS 생성 툴을 찾을 수 없습니다: {NVS_GEN_TOOL}")
        print("ESP-IDF 환경이 올바르게 로드되었는지 확인하세요.")
        sys.exit(1)

def ensure_csv_exists():
    if not os.path.exists(CSV_FILE):
        print(f"\n[안내] {CSV_FILE} 파일이 없습니다. 기본 샘플 빈 파일을 생성합니다.")
        with open(CSV_FILE, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['SERIAL_NUM', 'APP_REG_NUM', 'USED_TIME'])
            writer.writerow(['DEV001', 'ABC01DEF', ''])
            writer.writerow(['DEV002', 'XYZ98765', ''])
        print(f"-> {CSV_FILE} (엑셀) 에 생산할 시리얼과 등록번호를 기입하고 다시 실행해주세요.")
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
                
            serial_num, app_reg_num, used_time = row[0].strip(), row[1].strip(), row[2].strip()
            
            if not used_time and selected_index == -1:
                selected_index = i + 1 
            
            rows.append(row)
            
    if selected_index == -1:
        print("\n[완료] CSV 파일 내의 모든 시리얼 넘버가 이미 사용되었습니다!")
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
    idf_python_env = os.environ.get('IDF_PYTHON_ENV_PATH')
    if idf_python_env:
        python_exe = os.path.join(idf_python_env, 'Scripts', 'python.exe')
    else:
        python_exe = r'C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe'

    if not os.path.exists(python_exe):
        python_exe = sys.executable

    cmd = [python_exe, NVS_GEN_TOOL, 'generate', NVS_CSV_FILE, NVS_BIN_FILE, '0x6000']
    try:
        subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return True
    except subprocess.CalledProcessError as e:
        print("\n[에러] NVS 바이너리 생성 실패:")
        print(e.stderr.decode('utf-8'))
        return False

def flash_all_files(port):
    """폴더 내 존재하는 모든 펌웨어 5종 + 생성된 nvs.bin 1종을 합쳐서 한 번에 굽습니다."""
    flash_args = []
    
    print("\n[파일 점검 중]")
    for filename, offset in BIN_MAP.items():
        file_path = os.path.join(SCRIPT_DIR, filename)
        if os.path.exists(file_path):
            flash_args.extend([offset, file_path])
            print(f" - {filename:<25} -> {offset}")
        else:
            print(f" - [건너뜀] {filename:<13} -> 파일을 폴더에서 찾을 수 없습니다.")
    
    if not flash_args:
        print("\n[에러] 구울 펌웨어 바이너리가 1개도 없습니다!")
        return False
        
    idf_python_env = os.environ.get('IDF_PYTHON_ENV_PATH')
    if idf_python_env:
        python_exe = os.path.join(idf_python_env, 'Scripts', 'python.exe')
    else:
        python_exe = r'C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe'

    if not os.path.exists(python_exe):
        python_exe = sys.executable

    cmd = [python_exe, '-m', 'esptool', '--port', port, '--baud', '921600', 'write_flash'] + flash_args
    try:
        print(f"\n-> 포트 {port} 을(를) 통해 전체 {len(flash_args)//2}개 파일 굽기를 시작합니다...")
        print("   (장치가 재부팅되지 않았거나 권한이 없으면 실패할 수 있습니다. 수 초간 기다려주세요.)\n")
        
        # subprocess.run 대신 실시간 출력을 보여주기 위해 Popen 사용
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:
            sys.stdout.write("   " + line) # 약간 들여쓰기 출력
        proc.wait()
        
        if proc.returncode == 0:
            return True
        else:
            print(f"\n[에러] 플래싱 프로세스가 {proc.returncode} 상태로 종료되었습니다.")
            return False
            
    except Exception as e:
        print(f"\n[에러] 플래시에 굽기 실패! 보드가 연결되었고 COM 포트가 맞는지 확인하세요. ({e})")
        return False

def main():
    parser = argparse.ArgumentParser(description="Movition Factory Multi-Flash Tool")
    parser.add_argument('-p', '--port', default='COM4', help='기기가 연결된 COM 포트 (기본: COM4)')
    args = parser.parse_args()

    check_environment()
    ensure_csv_exists()

    print("\n==============================================")
    print("      Movition 공장 통합 양산 자동화 툴      ")
    print("==============================================")
    print("현재 실행 폴더:", SCRIPT_DIR)
    print(f"연결 포트: {args.port} (변경시: python factory_flash.py -p COM5)\n")

    while True:
        rows, target_idx = get_next_unused_serial()
        target_serial = rows[target_idx][0]
        target_reg = rows[target_idx][1]

        print(f"\n==============================================")
        print(f" [대기 중] 기기 시리얼 : '{target_serial}'")
        print(f"           앱 등록번호 : '{target_reg}'")
        print(f"==============================================")
        
        cmd = input("\n👉 새 기기를 USB에 연결하고 [Enter]를 치세요. (종료: 'q'): ")
        if cmd.lower() == 'q':
            break

        # 1. NVS 자동 생성
        generate_nvs_csv(target_serial, target_reg)
        if not generate_nvs_bin():
            print("NVS 파일 생성에 실패하여 진행할 수 없습니다.")
            continue
        
        # 2. 통합 굽기 진행
        if flash_all_files(args.port):
            # 3. CSV 저장 완료 마킹
            mark_as_used(rows, target_idx)
            print(f"\n✅ [성공] '{target_serial}' 시리얼 주입 및 통합 펌웨어 굽기 완료!")
            print(f"   (serial.csv의 {target_idx+1}번째 행에 사용시간이 기록되었습니다.)")
        else:
            print(f"\n❌ [실패] 굽기 도중 에러가 발생했습니다. 케이블 포트(${args.port})와 접촉을 다시 확인하세요.")

if __name__ == '__main__':
    main()
