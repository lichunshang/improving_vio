from __future__ import print_function
from collections import OrderedDict
import os
import sys
import json
import numpy as np

dataset = sys.argv[1]
mode = sys.argv[2]
results_path = os.path.abspath(sys.argv[3])

tumvio_seqs = ["room1", "room2", "room3", "room4", "room5", "room6"]
lengths = [8, 16, 24, 32, 40, 48]

print("dataset: %s" % dataset)
print("mode: %s" % mode)
print("results_path: %s" % results_path)

json_data = OrderedDict()
stats_dump_file = open(os.path.join(results_path, "stats_dump.txt"), "a")

if mode == "single_run":
    seq_results_dirs = sorted([d for d in os.listdir(results_path) if os.path.isdir(os.path.join(results_path, d))])
    for length in lengths:
        trans_errors = []
        rot_errors = []
        for d in seq_results_dirs:
            if dataset == "tumvio" and any(s in d for s in tumvio_seqs):
                print("Skipping directory: %s" % d)
                continue

            trans_error = np.load(os.path.join(results_path, d, "rpe_%dm_trans.npy" % length))
            rot_error = np.load(os.path.join(results_path, d, "rpe_%dm_rot.npy" % length))
            trans_errors += list(trans_error)
            rot_error += list(rot_error)

        trans_rmse = np.sqrt(np.mean(np.square(trans_errors)))
        rot_rmse = np.sqrt(np.mean(np.square(rot_errors)))

        print("rmse_%dm_trans: %.5f" % (length, trans_rmse))
        print("rmse_%dm_rot:   %.5f" % (length, rot_rmse))

        json_data["rmse_%dm_trans" % length] = trans_rmse
        json_data["rmse_%dm_rot" % length] = rot_rmse

        stats_dump_file.write("%.10f %.10f " % (trans_rmse, rot_rmse))

    json_dump_file = open(os.path.join(results_path), "w")
    json.dump(json_data, json_dump_file)
    json_dump_file.close()
    stats_dump_file.close()

elif mode == "multi_run":
    run_results_dirs = sorted([d for d in os.listdir(results_path) if
                               os.path.isdir(os.path.join(results_path, d)) and d.startswith("run_")])
    stats = []
    for d in run_results_dirs:
        stats_dump_file = os.path.join(results_path, d, "stats_dump.txt")
        stats.append(np.loadtxt(stats_dump_file))

    stats = np.array(stats)
    ave_stats = np.mean(stats, axis=1)

    ave_stats_dump_file = open(os.path.join(results_path, "stats_dump.txt"), "w")
    ave_stats_dump_file.write(" ".join(["%.10f" % i for i in ave_stats]))
    ave_stats_dump_file.close()
