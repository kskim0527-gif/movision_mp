import os
import sys
import shutil
from datetime import datetime

def package(board_name="hd1"):
    version = datetime.now().strftime("%y%m%d")
    update_root = r"C:\vscode\movition_ws\update"
    
    # Custom naming logic as per user request
    if board_name == "0223":
        folder_name = f"update_movision_v{version}"
    elif board_name in ["ws", "hd1"]:
        folder_name = f"update_movision_ws_v{version}"
    else:
        folder_name = f"update_movision-{board_name}_v{version}"
        
    target_dir = os.path.join(update_root, folder_name)
    build_dir = r"C:\vscode\movition_ws\build"

    if not os.path.exists(target_dir):
        os.makedirs(target_dir, exist_ok=True)
        print(f"Created directory: {target_dir}")

    files_to_copy = [
        (os.path.join(build_dir, "bootloader", "bootloader.bin"), "bootloader.bin"),
        (os.path.join(build_dir, "partition_table", "partition-table.bin"), "partition-table.bin"),
        (os.path.join(build_dir, "ota_data_initial.bin"), "ota_data_initial.bin"),
        (os.path.join(build_dir, "movision.bin"), "movision.bin"),
        (os.path.join(build_dir, "storage.bin"), "storage.bin"),
    ]

    # Additional Tools to include in the package (excluding update_tool.html for special handling)
    tools_to_copy = [
        "ble_ota_tool.py",
        "ble_update.bat",
        "esptool_web.js",
    ]

    for src, dst_name in files_to_copy:
        dst = os.path.join(target_dir, dst_name)
        if os.path.exists(src):
            shutil.copy2(src, dst)
            print(f"Copied {src} to {dst}")
        else:
            print(f"Warning: Source file not found: {src}")

    # Copy Tools (excluding update_tool.html, which is handled separately)
    for tool_name in tools_to_copy:
        src = os.path.join(update_root, tool_name)
        dst = os.path.join(target_dir, tool_name)
        if os.path.exists(src):
            shutil.copy2(src, dst)
            print(f"Copied tool: {tool_name}")
        else:
            print(f"Warning: Tool not found: {src}")

    # Create readme.txt (renamed from 참고사항.txt for consistency)
    memo_path = os.path.join(target_dir, "readme.txt")
    memo_content = f"""모델명 : MOVISION-{board_name.upper()}
F/W : {version}
날짜 : {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}

[Flash Addresses]
bootloader.bin\t0x0
partition-table.bin\t0x8000
ota_data_initial.bin\t0xF000
movision.bin (Main)\t0x20000
storage.bin\t0x3b2000
"""
    with open(memo_path, "w", encoding="utf-8") as f:
        f.write(memo_content)
    print(f"Created {memo_path}")

    # Special handling for update_tool.html to make it fresh and working
    html_src = os.path.join(update_root, "update_tool.html")
    if os.path.exists(html_src):
        with open(html_src, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 1. Update project file name (from 'update.bin' to 'movision.bin')
        content = content.replace("'update.bin'", "'movision.bin'")
        
        # 2. Update version/date text in the UI
        date_str = datetime.now().strftime("%Y-%m-%d")
        version_text = f"Version {version} ({date_str})"
        old_subtitle = '<p class="subtitle">모비전 펌웨어 및 데이터를 안전하게 전송합니다.</p>'
        new_subtitle = f'<p class="subtitle">모비전 펌웨어 전송 도구 - {version_text}</p>'
        content = content.replace(old_subtitle, new_subtitle)
        
        # 3. Write to target folder (this creates a 'fresh' file timestamp)
        html_dst = os.path.join(target_dir, "update_tool.html")
        with open(html_dst, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"Updated and copied {html_dst}")
    
    print(f"\n[SUCCESS] Package created in {target_dir}")

if __name__ == "__main__":
    board = sys.argv[1] if len(sys.argv) > 1 else "hd1"
    package(board)
