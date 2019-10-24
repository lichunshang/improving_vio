import sys
import os

path = os.path.abspath(sys.argv[1])
path_out = os.path.join(os.path.dirname(path), "data_tum.csv")

print("Converting %s to data_tum.csv" % path)
file_in = open(path, "r")
file_out = open(path_out, "w")

line = file_in.readline()
file_out.write("#timestamp p_WS_W_x p_WS_W_y p_WS_W_z q_WS_x q_WS_y q_WS_z q_WS_w\n")

for line in file_in:
    line = line.split(",")
    line_out = []
    line_out.append("%.9f" % (int(line[0]) / 1e9))
    for i in range(1, 8):
        line_out.append(line[i].strip())
    q = line_out[4:8]
    q_out = [q[1], q[2], q[3], q[0]]  # tum uses xyzw
    line_out[4:8] = q_out
    file_out.write(" ".join(line_out) + "\n")

file_in.close()
file_out.close()
print("Done.")
