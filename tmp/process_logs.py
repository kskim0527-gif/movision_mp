import os
import re

input_dir = r"C:\Users\kskim\내 드라이브\JOB\BSD-round\field_log\2026-03-18"
output_file = r"C:\Users\kskim\내 드라이브\JOB\BSD-round\field_log\2026-03-18_processed.txt"

# Get all txt files in the directory, sorted by name
files = sorted([f for f in os.listdir(input_dir) if f.endswith(".txt")])

with open(output_file, 'w', encoding='utf-8') as outfile:
    for filename in files:
        filepath = os.path.join(input_dir, filename)
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as infile:
            for line in infile:
                # Example line: "[12:24:34] [RX] 4D 04 01 00 // comment"
                
                # 1. Remove comments
                if "//" in line:
                    line = line.split("//")[0]
                
                # 2. Extract hex data (starting with 4D)
                match = re.search(r"4D\s.*", line)
                if match:
                    hex_data = match.group(0).strip()
                    if hex_data:
                        # 3. Format as "19 [data] 2F"
                        formatted_line = f"19 {hex_data} 2F\n"
                        outfile.write(formatted_line)

print(f"Processed into {output_file}")
