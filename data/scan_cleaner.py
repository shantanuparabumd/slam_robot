import re
import numpy as np

input_file = 'scan_log.txt'
output_file = 'scan_clean_data.txt'

# Open the input file
with open(input_file, 'r') as f:
    lines = f.readlines()

# Define the regular expression patterns
data_pattern = r'ranges=\[(.*?)\]'

# Extract values between 'ranges=[ ' and '] '
extracted_values = []
for line in lines:
    data_pattern_matches = re.findall(data_pattern, line)
    
    if data_pattern_matches:
        ranges_str = data_pattern_matches[0]
        ranges = [float(value) if value != 'inf' else np.inf for value in ranges_str.split(',')]
        # Convert 'inf' strings to max range
        extracted_values.append(','.join(map(str, ranges)) + '\n')

# Write the extracted values to the output file
with open(output_file, 'w') as f:
    f.writelines(extracted_values)

print(f'Extracted values concatenated and saved to {output_file}')
