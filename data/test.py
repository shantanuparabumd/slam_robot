nested_list = [[1, 2, 3, 4, 5],
               [6, 7, 8, 9, 10],
               [11, 12, 13, 14, 15]]

flattened_list = [element for sublist in nested_list[:2] for element in sublist]

print(flattened_list)
