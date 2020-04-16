import numpy as np
import matplotlib.pyplot as plt

# seq = "V1_01_easy"
# seq = "V1_02_medium"
seq = "V1_03_difficult"
# seq = "V2_02_medium"
# seq = "V2_03_difficult"
# seq = "MH_01_easy"
# seq = "MH_05_difficult"
path_vanilla = "/home/cs4li/Dev/dump/final_thesis_results_dense/feature_stats_EUROC_vanilla/run_1/%s_vins_feature_stats.txt" % seq
path_dense = "/home/cs4li/Dev/dump/final_thesis_results_dense/feature_stats_EUROC_dense/run_1/%s_vins_feature_stats.txt" % seq

data_vanilla = np.loadtxt(path_vanilla)
data_dense = np.loadtxt(path_dense)

plt.figure(1)
bins = np.linspace(0, 1, 50)
plt.hist(data_vanilla[:, 2], bins, alpha=0.4, label='VINS-Mono KLT', color="b")
plt.hist(data_dense[:, 2], bins, alpha=0.4, label='VINS-Mono PWC-Net', color="r")
plt.xlabel("inlier ratio")
plt.ylabel("frames count")
# plt.grid()
plt.legend(loc='upper left')

plt.savefig("/home/cs4li/Dev/dump/final_thesis_results_dense/figures/hist_%s_inlier.pdf" % seq,  format='pdf', bbox_inches='tight', pad_inches=0)

plt.figure(2)
bins = np.linspace(0, 150, 50)
plt.hist(data_vanilla[:, 1], bins, alpha=0.4, label='VINS-Mono KLT', color="b")
plt.hist(data_dense[:, 1], bins, alpha=0.4, label='VINS-Mono PWC-Net', color="r")
plt.xlabel("number of features per frame")
plt.ylabel("frames count")
# plt.grid()
plt.legend(loc='upper left')

plt.savefig("/home/cs4li/Dev/dump/final_thesis_results_dense/figures/hist_%s_count.pdf" % seq,  format='pdf', bbox_inches='tight', pad_inches=0)

# plt.show()
