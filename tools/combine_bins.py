import sys
import struct
import os

def combine_bins(app_bin_path, storage_bin_path, output_bin_path):
    if not os.path.exists(app_bin_path):
        print(f"Error: App binary not found at {app_bin_path}")
        return
    if not os.path.exists(storage_bin_path):
        print(f"Error: Storage binary not found at {storage_bin_path}")
        return

    with open(app_bin_path, 'rb') as f:
        app_data = f.read()

    with open(storage_bin_path, 'rb') as f:
        storage_data = f.read()

    app_size = len(app_data)
    storage_size = len(storage_data)
    magic = b'UPGD' # 0x44475055
    reserved = b'\x00\x00\x00\x00'

    # Header format: Magic(4), AppSize(4), StorageSize(4), Reserved(4) = 16 bytes
    header = struct.pack('<4sIII', magic, app_size, storage_size, 0)

    with open(output_bin_path, 'wb') as f:
        f.write(header)
        f.write(app_data)
        f.write(storage_data)

    print(f"Successfully created {output_bin_path}")
    print(f"  App Size: {app_size} bytes")
    print(f"  Storage Size: {storage_size} bytes")
    print(f"  Total Size: {os.path.getsize(output_bin_path)} bytes")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python combine_bins.py <app_bin> <storage_bin> <output_bin>")
        # Default paths if run from project root or tools dir
        base_dir = os.getcwd()
        if os.path.basename(base_dir) == 'tools':
            base_dir = os.path.dirname(base_dir)
        
        app_bin = os.path.join(base_dir, 'build', 'movision.bin')
        storage_bin = os.path.join(base_dir, 'build', 'storage.bin')
        output_bin = os.path.join(base_dir, 'movision_update.bin')
        
        print(f"Using default paths:\n  App: {app_bin}\n  Storage: {storage_bin}\n  Output: {output_bin}")
        combine_bins(app_bin, storage_bin, output_bin)
    else:
        combine_bins(sys.argv[1], sys.argv[2], sys.argv[3])
