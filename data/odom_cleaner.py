import re

input_file = 'odom_log.txt'
output_file = 'odom_clean_data.txt'

# Open the input file
with open(input_file, 'r') as f:
    lines = f.readlines()

# Define the regular expression patterns
quaternion_pattern = r'Quaternion\(x=(.*?), y=(.*?), z=(.*?), w=(.*?)\)'
point_pattern = r'Point\(x=(.*?), y=(.*?), z=(.*?)\)'

# Extract x, y, z, and w values from 'Quaternion(' and 'Point(' substrings using regex
extracted_values = []
for line in lines:
    quaternion_matches = re.findall(quaternion_pattern, line)
    point_matches = re.findall(point_pattern, line)
    
    if quaternion_matches and point_matches:
        x, y, z, w = quaternion_matches[0]
        x_point, y_point, z_point = point_matches[0]
        extracted_values.append(f'{x_point},{y_point},{z_point},{x},{y},{z},{w}\n')

# Write the extracted values to the output file
with open(output_file, 'w') as f:
    f.writelines(extracted_values)

print(f'Extracted values concatenated and saved to {output_file}')