import os
from PIL import Image

def convert_folder(folder):
    if not os.path.exists(folder):
        return
    for filename in os.listdir(folder):
        if filename.lower().endswith('.png') or filename.lower().endswith('.jpg'):
            path = os.path.join(folder, filename)
            try:
                with Image.open(path) as img:
                    print(f"Processing {filename}: {img.size}, {img.mode}")
                    res_img = img.convert('RGB')
                    res_img.save(path, 'PNG' if filename.lower().endswith('.png') else 'JPEG', optimize=True)
                    print(f"  Optimized {filename}")
            except Exception as e:
                print(f"  Error processing {filename}: {e}")

convert_folder(r'C:\vscode\movition_ws\main\flash_data\clock_1')
convert_folder(r'C:\vscode\movition_ws\main\flash_data\clock_2')
convert_folder(r'C:\vscode\movition_ws\main\flash_data\image')
convert_folder(r'C:\vscode\movition_ws\main\flash_data\photo')
