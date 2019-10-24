#!/usr/bin/env bash
set -e
trap 'exit 130' INT

function run() {
    echo "*** Launch vins estimator"
    roslaunch vins_estimator tum.launch &
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

run /home/cs4li/Dev/TUMVIO/bags/dataset-room1_512_16.bag room1
run /home/cs4li/Dev/TUMVIO/bags/dataset-room2_512_16.bag room2
run /home/cs4li/Dev/TUMVIO/bags/dataset-room3_512_16.bag room3
run /home/cs4li/Dev/TUMVIO/bags/dataset-room4_512_16.bag room4
run /home/cs4li/Dev/TUMVIO/bags/dataset-room5_512_16.bag room5
run /home/cs4li/Dev/TUMVIO/bags/dataset-room6_512_16.bag room6

run /home/cs4li/Dev/TUMVIO/bags/dataset-corridor1_512_16.bag corridor1
run /home/cs4li/Dev/TUMVIO/bags/dataset-corridor2_512_16.bag corridor2
run /home/cs4li/Dev/TUMVIO/bags/dataset-corridor3_512_16.bag corridor3
run /home/cs4li/Dev/TUMVIO/bags/dataset-corridor4_512_16.bag corridor4
run /home/cs4li/Dev/TUMVIO/bags/dataset-corridor5_512_16.bag corridor5

run /home/cs4li/Dev/TUMVIO/bags/dataset-magistrale1_512_16.bag magistrale1
run /home/cs4li/Dev/TUMVIO/bags/dataset-magistrale2_512_16.bag magistrale2
run /home/cs4li/Dev/TUMVIO/bags/dataset-magistrale3_512_16.bag magistrale3
run /home/cs4li/Dev/TUMVIO/bags/dataset-magistrale4_512_16.bag magistrale4
run /home/cs4li/Dev/TUMVIO/bags/dataset-magistrale5_512_16.bag magistrale5
run /home/cs4li/Dev/TUMVIO/bags/dataset-magistrale6_512_16.bag magistrale6

run /home/cs4li/Dev/TUMVIO/bags/dataset-outdoors1_512_16.bag outdoors1
run /home/cs4li/Dev/TUMVIO/bags/dataset-outdoors2_512_16.bag outdoors2
run /home/cs4li/Dev/TUMVIO/bags/dataset-outdoors3_512_16.bag outdoors3
run /home/cs4li/Dev/TUMVIO/bags/dataset-outdoors4_512_16.bag outdoors4
run /home/cs4li/Dev/TUMVIO/bags/dataset-outdoors5_512_16.bag outdoors5
run /home/cs4li/Dev/TUMVIO/bags/dataset-outdoors6_512_16.bag outdoors6
run /home/cs4li/Dev/TUMVIO/bags/dataset-outdoors7_512_16.bag outdoors7
run /home/cs4li/Dev/TUMVIO/bags/dataset-outdoors8_512_16.bag outdoors8

run /home/cs4li/Dev/TUMVIO/bags/dataset-slides1_512_16.bag slides1
run /home/cs4li/Dev/TUMVIO/bags/dataset-slides2_512_16.bag slides2
run /home/cs4li/Dev/TUMVIO/bags/dataset-slides3_512_16.bag slides3