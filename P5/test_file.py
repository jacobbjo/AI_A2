from Common.functions import *
file = "P5_path.txt"

ost = read_from_file(file)
print(len(ost))

for row in ost[1]:
    print(row)
