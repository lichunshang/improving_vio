import os
import sys
import json

multirun_path = os.path.abspath(sys.argv[2])
dataset = sys.argv[1]

tumvio_seqs = ["room1", "room2", "room3", "room4", "room5", "room6"]

print "dataset: %s" % dataset
print "multirun_path: %s" % multirun_path

for d in os.listdir(multirun_path):
    if not d.startswith("run_"):
        continue

    results_dir = os.path.join(multirun_path, d)
    json_data = json.load(open(os.path.join(results_dir, "stats.json"), "r"))

    for k in json_data.items():
        pass
