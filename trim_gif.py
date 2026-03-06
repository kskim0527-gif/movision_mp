import sys

def get_frame_info(filepath):
    with open(filepath, 'rb') as f:
        data = f.read()
    
    frames = []
    pos = 13 # Skip header and Logical Screen Descriptor
    
    # Global Color Table
    if data[10] & 0x80:
        gct_size = 3 * (1 << ((data[10] & 0x07) + 1))
        pos += gct_size
    
    while pos < len(data):
        start_pos = pos
        block_type = data[pos]
        if block_type == 0x21: # Extension
            ext_type = data[pos+1]
            if ext_type == 0xF9: # Graphic Control Extension
                delay = data[pos+4] | (data[pos+5] << 8)
                frames.append({'start': pos, 'delay': delay})
            pos += 2
            while pos < len(data):
                block_size = data[pos]
                pos += block_size + 1
                if block_size == 0:
                    break
        elif block_type == 0x2C: # Image Descriptor
            pos += 10
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
    return frames

def trim_gif(filepath, output_path, num_frames_to_keep):
    with open(filepath, 'rb') as f:
        full_data = f.read()
        
    frames = get_frame_info(filepath)
    if num_frames_to_keep >= len(frames):
        print("Nothing to trim.")
        return

    # The end of the N-th frame is the start of the (N+1)-th Graphic Control Extension (or next block)
    # Actually, we should find the start of the frame we want to REMOVE
    cutoff_pos = frames[num_frames_to_keep]['start']
    
    trimmed_data = full_data[:cutoff_pos] + b'\x3B'
    
    with open(output_path, 'wb') as f:
        f.write(trimmed_data)
    print(f"Trimmed GIF saved to {output_path}. Kept {num_frames_to_keep} frames.")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        info = get_frame_info(sys.argv[1])
        for i, f in enumerate(info):
            print(f"Frame {i}: Pos {f['start']}, Delay {f['delay']*10}ms")
    else:
        trim_gif(sys.argv[1], sys.argv[1], int(sys.argv[2]))
