from __future__ import print_function
import evo.tools.file_interface as file_interface
import evo.core.sync as sync
import evo.core.trajectory as trajectory
import evo.core.metrics as metrics
import evo.core.filters as filters
import os
import sys
import json
from collections import OrderedDict
import numpy as np


def write(f, s):
    print(s)
    f.write(s + "\n")


gt_path = os.path.abspath(sys.argv[1])
est_path = os.path.abspath(sys.argv[2])
dump_dir_path = os.path.abspath(sys.argv[3])
log_path = os.path.abspath(sys.argv[4])
json_data = OrderedDict()

dump_json_file = open(os.path.join(dump_dir_path, "stats.json"), "w")
log_file = open(log_path, "a")

write(log_file, "================ evo_traj_full_eval =================")
write(log_file, "gt_path " + gt_path)
write(log_file, "est_path " + est_path)
write(log_file, "dump_dir_path " + dump_dir_path)
write(log_file, "log_path " + log_path)

gt_traj = file_interface.read_tum_trajectory_file(gt_path)
est_traj = file_interface.read_tum_trajectory_file(est_path)

traj_gt_synced, traj_est_synced = sync.associate_trajectories(gt_traj, est_traj, max_diff=0.01)
traj_est_aligned = trajectory.align_trajectory(traj_est_synced, traj_gt_synced, correct_scale=False,
                                               correct_only_scale=False)

# ape translation only metric
metric = metrics.APE(metrics.PoseRelation.translation_part)
metric.process_data((traj_gt_synced, traj_est_aligned,))
stat = metric.get_statistic(metrics.StatisticsType.rmse)
write(log_file, "ape_trans: %.5f" % stat)
json_data["ape_trans"] = stat

# ape rotation metric
metric = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
metric.process_data((traj_gt_synced, traj_est_aligned,))
stat = metric.get_statistic(metrics.StatisticsType.rmse)
write(log_file, "ape_rot_deg: %.5f" % stat)
json_data["ape_rot_deg"] = stat

# rpe metrics at different length
lengths = [8, 16, 24, 32, 40, 48]
for length in lengths:
    trans_metric = metrics.RPE(metrics.PoseRelation.translation_part,
                               delta=length, delta_unit=metrics.Unit.meters, all_pairs=True)
    rot_metric = metrics.RPE(metrics.PoseRelation.rotation_angle_deg,
                             delta=length, delta_unit=metrics.Unit.meters, all_pairs=True)

    try:
        trans_metric.process_data((traj_gt_synced, traj_est_aligned,))
        rot_metric.process_data((traj_gt_synced, traj_est_aligned,))
        trans_stat = trans_metric.get_statistic(metrics.StatisticsType.rmse)
        rot_stat = rot_metric.get_statistic(metrics.StatisticsType.rmse)

        trans_metric_errors = trans_metric.error
        rot_metric_errors = rot_metric.error
    except filters.FilterException:
        trans_stat = np.nan
        rot_stat = np.nan
        trans_metric_errors = []
        rot_metric_errors = []

    write(log_file, "rpe_%dm_trans: %.5f" % (length, trans_stat))
    write(log_file, "rpe_%dm_rot:   %.5f" % (length, rot_stat))
    json_data["rpe_%dm_trans" % length] = trans_stat
    json_data["rpe_%dm_rot" % length] = rot_stat

    np.savetxt(os.path.join(dump_dir_path, "rpe_%dm_trans.txt" % length), trans_metric_errors)
    np.savetxt(os.path.join(dump_dir_path, "rpe_%dm_rot.txt" % length), rot_metric_errors)

json.dump(json_data, dump_json_file)
dump_json_file.close()
log_file.close()
