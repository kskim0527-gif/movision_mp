import os
import re

def is_ascii(s):
    return all(ord(c) < 128 for c in s)

def rename_to_ascii(folder):
    if not os.path.exists(folder):
        return
    
    # Mapping for common Korean terms in this project
    mapping = {
        '배경': 'screen',
        '분침': 'min',
        '시침': 'hour',
        '초침': 'sec',
        '중앙원': 'center',
        '바늘': 'hand',
        '숫자': 'num'
    }

    for root, dirs, files in os.walk(folder):
        for name in files:
            if not is_ascii(name):
                old_path = os.path.join(root, name)
                
                # Try to replace based on mapping
                new_name = name
                for ko, en in mapping.items():
                    new_name = new_name.replace(ko, en)
                
                # Remove any remaining non-ascii characters
                new_name = "".join([c if ord(c) < 128 else '_' for c in new_name])
                
                # Clean up multiple underscores
                new_name = re.sub(r'_{2,}', '_', new_name)
                
                new_path = os.path.join(root, new_name)
                
                # Check for collisions
                if os.path.exists(new_path) and old_path != new_path:
                    base, ext = os.path.splitext(new_name)
                    counter = 1
                    while os.path.exists(os.path.join(root, f"{base}_{counter}{ext}")):
                        counter += 1
                    new_path = os.path.join(root, f"{base}_{counter}{ext}")

                print(f"Renaming: {name} -> {os.path.basename(new_path)}")
                os.rename(old_path, new_path)

rename_to_ascii(r'C:\vscode\movition_ws\main\flash_data')
