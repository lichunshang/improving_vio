#!/usr/bin/env bash
set -e
trap 'exit 130' INT

function run() {
    echo "*** Launch vins estimator"
    roslaunch vins_estimator euroc.launch $3 &
    pid=$!
    echo "PID is ${pid}"
    sleep 2
    echo "*** playing bag"
    rosbag play $1
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

run /home/cs4li/Dev/EUROC/bags/MH_01_easy.bag MH_01 ${opt}
run /home/cs4li/Dev/EUROC/bags/MH_02_easy.bag MH_02 ${opt}
run /home/cs4li/Dev/EUROC/bags/MH_03_medium.bag MH_03 ${opt}
run /home/cs4li/Dev/EUROC/bags/MH_04_difficult.bag MH_04 ${opt}
run /home/cs4li/Dev/EUROC/bags/MH_05_difficult.bag MH_05 ${opt}

run /home/cs4li/Dev/EUROC/bags/V1_01_easy.bag V1_01 ${opt}
run /home/cs4li/Dev/EUROC/bags/V1_02_medium.bag V1_02 ${opt}
run /home/cs4li/Dev/EUROC/bags/V1_03_difficult.bag V1_03 ${opt}

run /home/cs4li/Dev/EUROC/bags/V2_01_easy.bag V2_01 ${opt}
run /home/cs4li/Dev/EUROC/bags/V2_02_medium.bag V2_02 ${opt}
run /home/cs4li/Dev/EUROC/bags/V2_03_difficult.bag V2_03 ${opt}