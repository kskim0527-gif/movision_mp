import sys

def count_frames(filepath):
    with open(filepath, 'rb') as f:
        data = f.read()
    
    count = 0
    pos = 13 # Skip header and Logical Screen Descriptor
    
    # Global Color Table
    if data[10] & 0x80:
        gct_size = 3 * (1 << ((data[10] & 0x07) + 1))
        pos += gct_size
    
    while pos < len(data):
        block_type = data[pos]
        if block_type == 0x21: # Extension
            ext_type = data[pos+1]
            if ext_type == 0xF9: # Graphic Control Extension
                count += 1
            pos += 2
            while pos < len(data):
                block_size = data[pos]
                pos += block_size + 1
                if block_size == 0:
                    break
        elif block_type == 0x2C: # Image Descriptor
            pos += 10 # 0x2C + Left(2) + Top(2) + Width(2) + Height(2) + Packed(1)
            # Local Color Table
            if data[pos-1] & 0x80:
                lct_size = 3 * (1 << ((data[pos-1] & 0x07) + 1))
                pos += lct_size
            pos += 1 # LZW Minimum Code Size
            while pos < len(data):
                block_size = data[pos]
                pos += block_size + 1
                if block_size == 0:
                    break
        elif block_type == 0x3B: # Trailer
            break
        else:
            pos += 1
    return count

if __name__ == "__main__":
    filepath = sys.argv[1]
    print(count_frames(filepath))
