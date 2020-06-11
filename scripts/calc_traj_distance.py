import se3
import numpy as np

f = "/home/cs4li/Dev/improving_vio/results/Vanilla_dataset_multiruns/EUROC_OKVIS/run_1/V1_03_difficult/gt.tum"

def get_distance(fn):
    t = open(fn)
    t.readline()

    positions = []
    for line in t:
        line = line.split()
        positions.append(np.array((float(line[1]), float(line[2]), float(line[3]))))

    sum = 0
    for i in range(0, len(positions) - 1):
        sum += np.linalg.norm(positions[i + 1] - positions[i])

    return sum

print(get_distance(f))