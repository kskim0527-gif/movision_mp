import struct
import os
import argparse

def pack_update(app_bin, storage_bin, output_bin):
    MAGIC = b"UPGD"
    
    app_size = os.path.getsize(app_bin) if app_bin else 0
    storage_size = os.path.getsize(storage_bin) if storage_bin else 0
    
    # Header: Magic(4), AppSize(4), StorageSize(4), Reserved(4)
    header = struct.pack("<4sIII", MAGIC, app_size, storage_size, 0)
    
    with open(output_bin, "wb") as out_f:
        out_f.write(header)
        
        if app_bin:
            with open(app_bin, "rb") as f:
                out_f.write(f.read())
        
        if storage_bin:
            with open(storage_bin, "rb") as f:
                out_f.write(f.read())
                
    print(f"Update package created: {output_bin}")
    print(f"  App size:     {app_size} bytes")
    print(f"  Storage size: {storage_size} bytes")
    print(f"  Total size:   {os.path.getsize(output_bin)} bytes")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--app", help="Path to application binary (movision.bin)", default="build/movision.bin")
    parser.add_argument("--storage", help="Path to storage binary (storage.bin)", default="build/storage.bin")
    parser.add_argument("--output", help="Output update package path", default="update/update.bin")
    args = parser.parse_args()
    
    pack_update(args.app, args.storage, args.output)
