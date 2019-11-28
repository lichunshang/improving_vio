from __future__ import print_function
from collections import OrderedDict
import os
import sys
import json
import numpy as np


def write(f, s):
    print(s)
    f.write(s + "\n")


dataset = sys.argv[1]
mode = sys.argv[2]
results_path = os.path.abspath(sys.argv[3])
log_path = os.path.abspath(sys.argv[4])
log_file = open(log_path, "a")

tumvio_seqs = ["room1", "room2", "room3", "room4", "room5", "room6"]
failure_trans_tolerance = {
    "euroc": 1.0,
    "tumvio": 1.0,
    # "uzh_fpv": 10.0
}
lengths = [8, 16, 24, 32, 40, 48]

write(log_file, "================= collect_errors =================")
write(log_file, "dataset: %s" % dataset)
write(log_file, "mode: %s" % mode)
write(log_file, "results_path: %s" % results_path)

json_data = OrderedDict()
stats_dump_file = open(os.path.join(results_path, "stats_dump.txt"), "a")

if mode == "single_run":
    seq_results_dirs = sorted([d for d in os.listdir(results_path) if os.path.isdir(os.path.join(results_path, d))])
    for length in lengths:
        trans_errors = []
        rot_errors = []
        for d in seq_results_dirs:
            if dataset == "tumvio" and not any(s in d for s in tumvio_seqs):
                write(log_file, "Skipping directory \"%s\" tumvio room only" % d)
                continue
            seq_stats = json.load(open(os.path.join(results_path, d, "stats.json"), "r"))

            if seq_stats["ape_trans"] > failure_trans_tolerance[dataset]:
                write(log_file, "Skipping directory \"%s\" failure detected!!!" % d)
                continue

            t = np.loadtxt(os.path.join(results_path, d, "rpe_%dm_trans.txt" % length))
            r = np.loadtxt(os.path.join(results_path, d, "rpe_%dm_rot.txt" % length))
            trans_errors += list(t)
            rot_errors += list(r)

            # write(log_file, np.sqrt(np.mean(np.square(r))))

        trans_rmse = np.sqrt(np.mean(np.square(trans_errors)))
        rot_rmse = np.sqrt(np.mean(np.square(rot_errors)))
        # trans_rmse = np.mean(trans_errors)
        # rot_rmse = np.mean(rot_errors)

        write(log_file, "rmse_%dm_trans: %.5f" % (length, trans_rmse))
        write(log_file, "rmse_%dm_rot:   %.5f" % (length, rot_rmse))

        json_data["rmse_%dm_trans" % length] = trans_rmse
        json_data["rmse_%dm_rot" % length] = rot_rmse

        stats_dump_file.write("%.10f %.10f " % (trans_rmse, rot_rmse))

    json_dump_file = open(os.path.join(results_path, "stats.json"), "w")
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
    ave_stats = np.mean(stats, axis=0)

    ave_stats_dump_file = open(os.path.join(results_path, "stats_dump.txt"), "w")
    ave_stats_dump_file.write(" ".join(["%.10f" % i for i in ave_stats]))
    ave_stats_dump_file.write(" ")
    ave_stats_dump_file.close()

    np.savetxt(os.path.join(results_path, "stats_collected_mat.txt"), stats, fmt='%.10f',)
