#!/usr/bin/env bash
set -e
trap 'exit 130' INT

function run() {
    echo $1
    echo "*** Launch vins estimator"
    uzh_fpv_configs_dir="/home/cs4li/Dev/catkin_ws/src/VINS-Mono/config/uzh_fpv"

    if [[ $1 =~ "indoor_45" ]]; then
        config_file="${uzh_fpv_configs_dir}/uzh_fpv_indoor_45_config.yaml"
    elif [[ $1 =~ "indoor_forward" ]]; then
        config_file="${uzh_fpv_configs_dir}/uzh_fpv_indoor_forward_config.yaml"
    elif [[ $1 =~ "outdoor_45" ]]; then
        config_file="${uzh_fpv_configs_dir}/uzh_fpv_outdoor_45_config.yaml"
    elif [[ $1 =~ "outdoor_forward" ]]; then
        config_file="${uzh_fpv_configs_dir}/uzh_fpv_outdoor_forward_config.yaml"
    else
        exit 1
    fi

    rate_factor=1.0
    if [[ $3 == "dense:=true" ]]; then
        rate_factor=0.333333
    fi

    roslaunch vins_estimator uzh_fpv.launch config_path:=${config_file} $3 &
    pid=$!
    echo "PID is ${pid}"
    sleep 2
    echo "*** playing bag"
    rosbag play $1 -r ${rate_factor}
    echo "*** Killing everything"
    kill -INT ${pid}
    rosnode kill vins_estimator || true
    rosnode kill feature_tracker || true
    rosnode kill feature_tracker_dense || true
    sleep 2
    echo "*** Change name"
    mv /home/cs4li/Dev/dump/vins_result_no_loop.csv /home/cs4li/Dev/dump/$2_vins_result_no_loop.csv
}

if [[ $1 == "--dense" ]]; then
opt="dense:=true"
else
opt="dense:=false"
fi
echo ${opt}

bags_dir="/home/cs4li/Dev/UZH_FPV/bags"

#run ${bags_dir}/indoor_45_1_snapdragon.bag             indoor_45_1 ${opt}
run ${bags_dir}/indoor_45_2_snapdragon_with_gt.bag     indoor_45_2 ${opt}
#run ${bags_dir}/indoor_45_3_snapdragon.bag             indoor_45_3 ${opt}
run ${bags_dir}/indoor_45_4_snapdragon_with_gt.bag     indoor_45_4 ${opt}
run ${bags_dir}/indoor_45_9_snapdragon_with_gt.bag     indoor_45_9 ${opt}
#run ${bags_dir}/indoor_45_11_snapdragon.bag            indoor_45_11 ${opt}
run ${bags_dir}/indoor_45_12_snapdragon_with_gt.bag    indoor_45_12 ${opt}
run ${bags_dir}/indoor_45_13_snapdragon_with_gt.bag    indoor_45_13 ${opt}
run ${bags_dir}/indoor_45_14_snapdragon_with_gt.bag    indoor_45_14 ${opt}
#run ${bags_dir}/indoor_45_16_snapdragon.bag            indoor_45_16 ${opt}

run ${bags_dir}/indoor_forward_3_snapdragon_with_gt.bag   indoor_forward_3 ${opt}
run ${bags_dir}/indoor_forward_5_snapdragon_with_gt.bag   indoor_forward_5 ${opt}
run ${bags_dir}/indoor_forward_6_snapdragon_with_gt.bag   indoor_forward_6 ${opt}
run ${bags_dir}/indoor_forward_7_snapdragon_with_gt.bag   indoor_forward_7 ${opt}
#run ${bags_dir}/indoor_forward_8_snapdragon.bag           indoor_forward_8 ${opt}
run ${bags_dir}/indoor_forward_9_snapdragon_with_gt.bag   indoor_forward_9 ${opt}
run ${bags_dir}/indoor_forward_10_snapdragon_with_gt.bag  indoor_forward_10 ${opt}
#run ${bags_dir}/indoor_forward_11_snapdragon.bag          indoor_forward_11 ${opt}
#run ${bags_dir}/indoor_forward_12_snapdragon.bag          indoor_forward_12 ${opt}

run ${bags_dir}/outdoor_45_1_snapdragon_with_gt.bag outdoor_45_1 ${opt}
#run ${bags_dir}/outdoor_45_2_snapdragon.bag         outdoor_45_2 ${opt}

run ${bags_dir}/outdoor_forward_1_snapdragon_with_gt.bag  outdoor_forward_1 ${opt}
#run ${bags_dir}/outdoor_forward_2_snapdragon.bag          outdoor_forward_2 ${opt}
run ${bags_dir}/outdoor_forward_3_snapdragon_with_gt.bag  outdoor_forward_3 ${opt}
run ${bags_dir}/outdoor_forward_5_snapdragon_with_gt.bag  outdoor_forward_5 ${opt}
#run ${bags_dir}/outdoor_forward_6_snapdragon.bag          outdoor_forward_6 ${opt}
#run ${bags_dir}/outdoor_forward_9_snapdragon.bag          outdoor_forward_9 ${opt}
#run ${bags_dir}/outdoor_forward_10_snapdragon.bag         outdoor_forward_10 ${opt}