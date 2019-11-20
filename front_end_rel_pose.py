import os
import transformations
import numpy as np
import se3
import matplotlib.pyplot as plt

np.set_printoptions(linewidth=1024)

rel_pose_est_path = "/home/cs4li/Dev/dump/feature_tracker_rel_pose.txt"
gt_tum_path = "/home/cs4li/Dev/TUMVIO/dataset-outdoors2_512_16/mav0/mocap0/data_tum.csv"
output_dir = "/home/cs4li/Dev/dump"

start_time = 1520430766.554749


# command = "rosbag play dataset-outdoors2_512_16.bag -s 875"

def plot(rel_est_data, rel_gt_data, gt_rel_se3s_no_est):
    labels = ["Trans X", "Trans Y", "Trans Z", "Rot X", "Rot Y", "Rot Z"]

    for j in range(1, 7):
        # rel estimates
        plt.figure(j)
        plt.clf()
        w = 0.5
        plt.plot(rel_est_data[:, 0] - start_time, rel_est_data[:, j], color="r", linewidth=w)
        plt.plot(rel_gt_data[:, 0] - start_time, rel_gt_data[:, j], color="b", linewidth=w)
        plt.plot(gt_rel_se3s_no_est[:, 0] - start_time, gt_rel_se3s_no_est[:, j], color="g", linewidth=w)
        plt.xlabel("frame # []")
        plt.ylabel(labels[j - 1].lower())
        plt.title("%s %s Plot" % ("Rel Gt", labels[j - 1]))
        plt.grid(which='both')
        plt.minorticks_on()
        plt.savefig(
                os.path.join(output_dir, "%s_%s_plt.svg" % ("rel", "_".join(labels[j - 1].lower().split()))),  format='svg', dpi=2000)

    # plt.show()

def find_bounding_index(gt_timestmaps, ts1, ts2):
    ts1_idx1 = np.nonzero(ts1 >= gt_timestamps)[0]
    assert len(ts1_idx1) > 0
    ts1_idx1 = ts1_idx1[-1]
    ts1_idx2 = ts1_idx1 + 1
    assert (gt_timestmaps[ts1_idx2] - gt_timestmaps[ts1_idx1] < 0.05)

    ts2_idx1 = np.nonzero(ts2 > gt_timestamps)[0]
    assert len(ts2_idx1) > 0
    ts2_idx1 = ts2_idx1[-1]
    ts2_idx2 = ts2_idx1 + 1
    assert (gt_timestmaps[ts2_idx2] - gt_timestmaps[ts2_idx1] < 0.05)

    return ts1_idx1, ts1_idx2, ts2_idx1, ts2_idx2


def find_gt_rel_pose(gt_timestamps, gt_poses, ts1, ts2):
    ts1_idx1, ts1_idx2, ts2_idx1, ts2_idx2 = find_bounding_index(gt_timestamps, ts1, ts2)
    ts1_alpha = (ts1 - gt_timestamps[ts1_idx1]) / (gt_timestamps[ts1_idx2] - gt_timestamps[ts1_idx1])
    ts2_alpha = (ts2 - gt_timestamps[ts2_idx1]) / (gt_timestamps[ts2_idx2] - gt_timestamps[ts2_idx1])

    assert ts1_alpha > 0
    assert ts2_alpha > 0

    ts1_T = se3.interpolate_SE3(gt_poses[ts1_idx1], gt_poses[ts1_idx2], ts1_alpha)
    ts2_T = se3.interpolate_SE3(gt_poses[ts2_idx1], gt_poses[ts2_idx2], ts2_alpha)

    rel_T = np.linalg.inv(ts1_T).dot(ts2_T)

    return rel_T


rel_poses_est_file = open(rel_pose_est_path, "r")
gt_tum_file = open(gt_tum_path, "r")
gt_timestamps = []
gt_poses = []
gt_tum_file.readline()

T_vc = np.array([-9.9951465899298464e-01, 7.5842033363785165e-03, -3.0214670573904204e-02, 4.4511917113940799e-02,
                 2.9940114644659861e-02, -3.4023430206013172e-02, -9.9897246995704592e-01, -7.3197096234105752e-02,
                 -8.6044170750674241e-03, -9.9939225835343004e-01, 3.3779845322755464e-02, -4.7972907300764499e-02,
                 0, 0, 0, 1]).reshape(4, 4)

for line in gt_tum_file:
    line = line.split()
    ts = float(line[0])
    x = float(line[1])
    y = float(line[2])
    z = float(line[3])
    qx = float(line[4])
    qy = float(line[5])
    qz = float(line[6])
    qw = float(line[7])
    T = transformations.quaternion_matrix([qw, qx, qy, qz])
    T[:3, 3] = np.array([x, y, z])
    gt_timestamps.append(ts)
    gt_poses.append(T)

gt_timestamps = np.array(gt_timestamps)

rel_poses_est_lines = []
for line in rel_poses_est_file:
    rel_poses_est_lines.append([float(l) for l in line.split()])

gt_rel_se3s = []
gt_rel_se3s_no_est = []
est_rel_se3s = []
for i in range(0, len(rel_poses_est_lines)):

    try:
        if len(rel_poses_est_lines[i]) == 1:
            print "No solution"
            est_rel_se3s.append(np.array([rel_poses_est_lines[i][0], np.nan, np.nan, np.nan, np.nan, np.nan, np.nan]))

            gt_rel_T = find_gt_rel_pose(gt_timestamps, gt_poses, rel_poses_est_lines[i][0] - 0.1, rel_poses_est_lines[i][0])
            gt_rel_T = np.linalg.inv(T_vc).dot(gt_rel_T.dot(T_vc))
            gt_rel_se3s_no_est.append(np.array([rel_poses_est_lines[i][0]] + list(se3.log_SE3(gt_rel_T))))
            gt_rel_se3s.append(np.array([rel_poses_est_lines[i][0], np.nan, np.nan, np.nan, np.nan, np.nan, np.nan]))
            continue
        else:
            gt_rel_se3s_no_est.append(np.array([rel_poses_est_lines[i][0], np.nan, np.nan, np.nan, np.nan, np.nan, np.nan]))

        gt_rel_T = find_gt_rel_pose(gt_timestamps, gt_poses, rel_poses_est_lines[i][1], rel_poses_est_lines[i][0])
        gt_rel_T = np.linalg.inv(T_vc).dot(gt_rel_T.dot(T_vc))

        gt_rel_se3s.append(np.array([rel_poses_est_lines[i][0]] + list(se3.log_SE3(gt_rel_T))))
    except AssertionError as e:
        print "Bad Assertion", e
        gt_rel_se3s.append(np.array([rel_poses_est_lines[i][0], np.nan, np.nan, np.nan, np.nan, np.nan, np.nan]))
        est_rel_se3s.append(np.array([rel_poses_est_lines[i][0], np.nan, np.nan, np.nan, np.nan, np.nan, np.nan]))
        continue

    est_rel_T = np.array(rel_poses_est_lines[i][2:])
    assert (len(est_rel_T) == 12)
    est_rel_T = est_rel_T.reshape(3, 4)
    est_rel_T = np.concatenate([est_rel_T, [[0, 0, 0, 1]]], axis=0)

    scale = np.linalg.norm(gt_rel_T[:3, 3]) / np.linalg.norm(est_rel_T[:3, 3])
    est_rel_T[:3, 3] = scale * est_rel_T[:3, 3]

    est_rel_se3s.append(np.array([rel_poses_est_lines[i][0]] + list(se3.log_SE3(est_rel_T))))

    # print "%.15f" % rel_poses_est_lines[i][0]
    # print "gt_rel_se3\n", gt_rel_se3s[-1]
    # print "est_rel_se3\n", est_rel_se3s[-1]
    # print "gt_rel_T\n", gt_rel_T
    # print "est_rel_T\n", est_rel_T
    # print

gt_rel_se3s = np.array(gt_rel_se3s)
est_rel_se3s = np.array(est_rel_se3s)
gt_rel_se3s_no_est = np.array(gt_rel_se3s_no_est)
plot(est_rel_se3s, gt_rel_se3s, gt_rel_se3s_no_est)

print("Done")
