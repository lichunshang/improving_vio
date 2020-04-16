from __future__ import print_function
import os
import sys
import numpy as np

path = sys.argv[1]
all_stats = np.zeros([0, 3])

if os.path.isdir(path):
    all_files = os.listdir(path)
    for f in all_files:
        if f.endswith("feature_stats.txt"):
            s = np.loadtxt(os.path.join(path, f))
            all_stats = np.concatenate([all_stats, s], axis=-1)
else:
    all_stats = np.loadtxt(path)

print("before => average: %f" % (np.mean(all_stats[:, 0])))
print("after => average: %f, median: %f" % (np.mean(all_stats[:, 1]), np.median(all_stats[:, 1])))
print("ratio => average: %f, median: %f" % (np.mean(all_stats[:, 2]), np.median(all_stats[:, 2])))
