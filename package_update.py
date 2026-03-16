import os
import shutil
from datetime import datetime

def package():
    version = datetime.now().strftime("%y%m%d")
    update_root = r"C:\vscode\movition_ws\update"
    folder_name = f"update_movision-hd1_v{version}"
    target_dir = os.path.join(update_root, folder_name)
    build_dir = r"C:\vscode\movition_ws\build"

    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
        print(f"Created directory: {target_dir}")

    files_to_copy = [
        (os.path.join(build_dir, "bootloader", "bootloader.bin"), "bootloader.bin"),
        (os.path.join(build_dir, "partition_table", "partition-table.bin"), "partition-table.bin"),
        (os.path.join(build_dir, "ota_data_initial.bin"), "ota_data_initial.bin"),
        (os.path.join(build_dir, "movision.bin"), "movision.bin"),
        (os.path.join(build_dir, "storage.bin"), "storage.bin"),
    ]

    for src, dst_name in files_to_copy:
        dst = os.path.join(target_dir, dst_name)
        if os.path.exists(src):
            shutil.copy2(src, dst)
            print(f"Copied {src} to {dst}")
        else:
            print(f"Warning: Source file not found: {src}")

    # Create 참고사항.txt
    memo_path = os.path.join(target_dir, "참고사항.txt")
    memo_content = f"""모델명 : MOVISION-HD1
F/W : {version}

bootloader.bin\t0x0\t
build/bootloader/bootloader.bin

partition-table.bin\t0x8000\t
build/partition_table/partition-table.bin

ota_data_initial.bin\t0xF000\t
build/ota_data_initial.bin

movision.bin (Main)\t0x20000\t
build/movision.bin

storage.bin\t0x3A4000\t
build/storage.bin
"""
    with open(memo_path, "w", encoding="utf-8") as f:
        f.write(memo_content)
    print(f"Created {memo_path}")

if __name__ == "__main__":
    package()
