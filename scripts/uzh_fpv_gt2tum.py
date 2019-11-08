import sys
import os

path = os.path.abspath(sys.argv[1])
path_out = os.path.join(os.path.dirname(path), "groundtruth_tum.txt")

print("Converting %s to groundtruth_tum.csv" % path)
file_in = open(path, "r")
file_out = open(path_out, "w")

line = file_in.readline()
file_out.write("#timestamp p_WS_W_x p_WS_W_y p_WS_W_z q_WS_x q_WS_y q_WS_z q_WS_w\n")

for line in file_in:
    line = line.strip().split(" ")
    line_out = line[1:9]
    file_out.write(" ".join(line_out) + "\n")

file_in.close()
file_out.close()
print("Done.")
