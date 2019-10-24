#!/usr/bin/env bash
set -e
trap 'exit 130' INT

function run() {
    echo "*** Launch vins estimator"
    roslaunch vins_estimator euroc.launch &
    pid=$!
    echo "PID is ${pid}"
    sleep 2
    echo "*** playing bag"
    rosbag play $1
    echo "*** Killing everything"
    kill -INT ${pid}
    rosnode kill -a
    sleep 2
    echo "*** Change name"
    mv /home/cs4li/Dev/dump/vins_result_no_loop.csv /home/cs4li/Dev/dump/$2_vins_result_no_loop.csv
}

run /home/cs4li/Dev/EUROC/bags/MH_01_easy.bag MH_01
run /home/cs4li/Dev/EUROC/bags/MH_02_easy.bag MH_02
run /home/cs4li/Dev/EUROC/bags/MH_03_medium.bag MH_03
run /home/cs4li/Dev/EUROC/bags/MH_04_hard.bag MH_04
run /home/cs4li/Dev/EUROC/bags/MH_05_hard.bag MH_05

run /home/cs4li/Dev/EUROC/bags/V1_01_easy.bag V1_01
run /home/cs4li/Dev/EUROC/bags/V1_02_medium.bag V1_02
run /home/cs4li/Dev/EUROC/bags/V1_03_hard.bag V1_03

run /home/cs4li/Dev/EUROC/bags/V2_01_easy.bag V2_01
run /home/cs4li/Dev/EUROC/bags/V2_02_medium.bag V2_02
run /home/cs4li/Dev/EUROC/bags/V2_03_hard.bag V2_03