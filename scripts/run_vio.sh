#!/usr/bin/env bash

declare -a euroc_seqs
euroc_seqs[0]="MH_01_easy"
euroc_seqs[1]="MH_02_easy"
euroc_seqs[2]="MH_03_medium"
euroc_seqs[3]="MH_04_difficult"
euroc_seqs[4]="MH_05_difficult"
euroc_seqs[5]="V1_01_easy"
euroc_seqs[6]="V1_02_medium"
euroc_seqs[7]="V1_03_difficult"
euroc_seqs[8]="V2_01_easy"
euroc_seqs[9]="V2_02_medium"
euroc_seqs[10]="V2_03_difficult"

declare -a tumvio_seqs
tumvio_seqs[0]="corridor1"
tumvio_seqs[1]="corridor2"
tumvio_seqs[2]="corridor3"
tumvio_seqs[3]="corridor4"
tumvio_seqs[4]="corridor5"

tumvio_seqs[5]="magistrale1"
tumvio_seqs[6]="magistrale2"
tumvio_seqs[7]="magistrale3"
tumvio_seqs[8]="magistrale4"
tumvio_seqs[9]="magistrale5"
tumvio_seqs[10]="magistrale6"

tumvio_seqs[11]="outdoors1"
tumvio_seqs[12]="outdoors2"
tumvio_seqs[13]="outdoors3"
tumvio_seqs[14]="outdoors4"
tumvio_seqs[15]="outdoors5"
tumvio_seqs[16]="outdoors6"
tumvio_seqs[17]="outdoors7"
tumvio_seqs[18]="outdoors8"

tumvio_seqs[19]="room1"
tumvio_seqs[20]="room2"
tumvio_seqs[21]="room3"
tumvio_seqs[22]="room4"
tumvio_seqs[23]="room5"
tumvio_seqs[24]="room6"

tumvio_seqs[25]="slides1"
tumvio_seqs[26]="slides2"
tumvio_seqs[27]="slides3"

declare -a uzh_fpv_seqs
uzh_fpv_seqs[0]="indoor_45_2"
uzh_fpv_seqs[1]="indoor_45_4"
uzh_fpv_seqs[2]="indoor_45_9"
uzh_fpv_seqs[3]="indoor_45_12"
uzh_fpv_seqs[4]="indoor_45_13"
uzh_fpv_seqs[5]="indoor_45_14"
uzh_fpv_seqs[6]="indoor_forward_3"
uzh_fpv_seqs[7]="indoor_forward_5"
uzh_fpv_seqs[8]="indoor_forward_6"
uzh_fpv_seqs[9]="indoor_forward_7"
uzh_fpv_seqs[10]="indoor_forward_9"
uzh_fpv_seqs[11]="indoor_forward_10"
uzh_fpv_seqs[12]="outdoor_45_1"
uzh_fpv_seqs[13]="outdoor_forward_1"
uzh_fpv_seqs[14]="outdoor_forward_3"
uzh_fpv_seqs[15]="outdoor_forward_5"


function run_vins_mono_ros() {
    bag=$2
    launch_file=$1
    dense_opt=$3
    config_opt=$4
    echo "*** Launch vins estimator"
    roslaunch vins_estimator ${launch_file} ${dense_opt} ${config_opt} &
    pid=$!
    echo "PID is ${pid}"
    sleep 2
    echo "*** playing bag"
    rosbag play ${bag}
    echo "*** Killing everything"
    kill -INT ${pid}
    rosnode kill vins_estimator || true
    rosnode kill feature_tracker || true
    rosnode kill feature_tracker_dense || true
    sleep 2
    echo "*** Change name"
    mv /home/cs4li/Dev/dump/vins_result_no_loop.csv /home/cs4li/Dev/dump/$2_vins_result_no_loop.csv
}

function fullseqname() {
    seq=$2
    if [[ $1 == "euroc" ]]; then
        echo ${seq}
    elif [[ $1 == "tumvio" ]]; then
        echo dataset-${key}_512_16
    elif [[ $1 == "uzh_fpv" ]]; then
        echo ${key}_snapdragon_with_gt
    else
        echo "full seq name error"
        exit 1
    fi
}

estimator=$1
dataset=$2
seqs=$3
dump_dir="/home/cs4li/Dev/dump"
uzh_fpv_configs_dir="/home/cs4li/Dev/catkin_ws/src/VINS-Mono/config/uzh_fpv"

cd ${dump_dir}

if [[ $4 == "--dense" ]]; then
    dense_opt="dense:=true"
else
    dense_opt="dense:=false"
fi

if [[ ${dataset} == "euroc" ]]; then
    seqs_to_run=${euroc_seqs[@]}
    dataset_dir="/home/cs4li/Dev/EUROC"
elif [[ ${dataset} == "tumvio" ]]; then
    seqs_to_run=${tumvio_seqs[@]}
    dataset_dir="/home/cs4li/Dev/TUMVIO"
elif [[ ${dataset} == "uzh_fpv" ]]; then
    seqs_to_run=${uzh_fpv_seqs[@]}
    dataset_dir="/home/cs4li/Dev/UZH_FPV"
else
    echo "dataset not valid"
    exit 1
fi

if [[ ${seqs} != "all" ]]; then
    declare -a seqs_to_run
    seqs_to_run[0]=${seqs}
fi

echo "estimator ${estimator}"
echo "dataset ${dataset}"
echo "seqs ${seqs}"
echo "dense_opt ${dense_opt}"

if [[ ${estimator} == "vins_mono" ]]; then
    for i in "${!seqs_to_run[@]}"
    do
        seq=${seqs_to_run[$i]}
        if [[ ${dataset} == "euroc" ]]; then
            launch_file="euroc.launch"
        elif [[ ${dataset} == "tumvio" ]]; then
            seqs_to_run=${tumvio_seqs[@]}
            launch_file="tum.launch"
        elif [[ ${dataset} == "uzh_fpv" ]]; then
            launch_file="uzh_fpv.launch"
            if [[ $seq =~ "indoor_45" ]]; then
                config_opt="${uzh_fpv_configs_dir}/uzh_fpv_indoor_45_config.yaml"
            elif [[ $seq =~ "indoor_forward" ]]; then
                config_opt="${uzh_fpv_configs_dir}/uzh_fpv_indoor_forward_config.yaml"
            elif [[ $seq =~ "outdoor_45" ]]; then
                config_opt="${uzh_fpv_configs_dir}/uzh_fpv_outdoor_45_config.yaml"
            elif [[ $seq =~ "outdoor_forward" ]]; then
                config_opt="${uzh_fpv_configs_dir}/uzh_fpv_outdoor_forward_config.yaml"
            else
                exit 1
            fi
        fi
        run_vins_mono_ros ${launch_file} ${dataset_dir}/bags/$(fullseqname ${dataset} ${seq}).bag ${dense_opt} ${config_opt}
    done
elif [[ ${estimator} == "okvis" ]]; then
    for i in "${!seqs_to_run[@]}"
    do
        seq=${seqs_to_run[$i]}
        if [[ ${dataset} == "uzh_fpv" ]]; then
            pass
        else
            config_yaml="/home/cs4li/Dev/catkin_ws/src/okvis_ros/okvis/config/config_okvis_50_20.yaml"
            config_yaml="/home/cs4li/Dev/catkin_ws/src/okvis_ros/okvis/config/config_fpga_p2_euroc.yaml"
            rosrun okvis_ros okvis_node_synchronous_from_file ${config_yaml} ${dataset_dir}/${seq}/mav0
        fi
    done
elif [[ ${estimator} == "uzh_fpv" ]]; then
    pass
else
    pass
fi