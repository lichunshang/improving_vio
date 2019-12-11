#!/usr/bin/env bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
evo_traj tum ${DIR}/est.tum --ref ${DIR}/gt.tum --save_plot ${seq_results_dir}/plot_overlay.pdf --plot --align
