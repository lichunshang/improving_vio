#!/usr/bin/env bash
set -e
trap 'exit 130' INT

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

declare -a uzh_fpv_seqs_w_gt
uzh_fpv_seqs_w_gt[0]="indoor_45_2"
uzh_fpv_seqs_w_gt[1]="indoor_45_4"
uzh_fpv_seqs_w_gt[2]="indoor_45_9"
uzh_fpv_seqs_w_gt[3]="indoor_45_12"
uzh_fpv_seqs_w_gt[4]="indoor_45_13"
uzh_fpv_seqs_w_gt[5]="indoor_45_14"
uzh_fpv_seqs_w_gt[6]="indoor_forward_3"
uzh_fpv_seqs_w_gt[7]="indoor_forward_5"
uzh_fpv_seqs_w_gt[8]="indoor_forward_6"
uzh_fpv_seqs_w_gt[9]="indoor_forward_7"
uzh_fpv_seqs_w_gt[10]="indoor_forward_9"
uzh_fpv_seqs_w_gt[11]="indoor_forward_10"
uzh_fpv_seqs_w_gt[12]="outdoor_45_1"
uzh_fpv_seqs_w_gt[13]="outdoor_forward_1"
uzh_fpv_seqs_w_gt[14]="outdoor_forward_3"
uzh_fpv_seqs_w_gt[15]="outdoor_forward_5"

declare -a uzh_fpv_seqs_no_gt
uzh_fpv_seqs_no_gt[0]="indoor_45_1"
uzh_fpv_seqs_no_gt[1]="indoor_45_3"
uzh_fpv_seqs_no_gt[2]="indoor_45_11"
uzh_fpv_seqs_no_gt[3]="indoor_45_16"
uzh_fpv_seqs_no_gt[4]="indoor_forward_8"
uzh_fpv_seqs_no_gt[5]="indoor_forward_11"
uzh_fpv_seqs_no_gt[6]="indoor_forward_12"
uzh_fpv_seqs_no_gt[7]="outdoor_45_2"
uzh_fpv_seqs_no_gt[8]="outdoor_forward_2"
uzh_fpv_seqs_no_gt[9]="outdoor_forward_6"
uzh_fpv_seqs_no_gt[10]="outdoor_forward_9"
uzh_fpv_seqs_no_gt[11]="outdoor_forward_10"

estimator=$1
dataset=$2
seqs=$3
this_script_dir="$( cd "$(dirname "$0")" ; pwd -P )"
results_dir=$4

dump_dir="/home/cs4li/Dev/dump"
vins_mono_uzh_fpv_configs_dir="/home/cs4li/Dev/catkin_ws/src/VINS-Mono/config/uzh_fpv"
okvis_uzh_fpv_configs_dir="/home/cs4li/Dev/catkin_ws/src/okvis_ros/okvis/config"
vinsmono2tum_script="${this_script_dir}/vinsmono_output2tum.py"
okvis2tum_script="${this_script_dir}/okvis_output2tum.py"
euroc_dataset_dir="/home/cs4li/Dev/EUROC"
tumvio_dataset_dir="/home/cs4li/Dev/TUMVIO"
uzh_fpv_dataset_dir="/home/cs4li/Dev/UZH_FPV"


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
}

function run_okvis_ros() {
    bag=$2
    launch_file=$1
    config_opt=$3
    echo "*** Launch vins estimator"
    echo ${launch_file}
    echo ${config_opt}
    roslaunch okvis_ros ${launch_file} ${config_opt} &
    pid=$!
    echo "PID is ${pid}"
    sleep 2
    echo "*** playing bag"
    rosbag play ${bag}
    echo "*** Killing everything"
    kill -INT ${pid}
    rosnode kill okvis_node || true
    sleep 2
}

array_contains() {
    local array="$1[@]"
    local seeking=$2
    local in=1
    for element in "${!array}"; do
        if [[ $element == "$seeking" ]]; then
            in=0
            break
        fi
    done
    return $in
}

function fullseqname() {
    seq=$2
    if [[ $1 == "euroc" ]]; then
        echo ${seq}
    elif [[ $1 == "tumvio" ]]; then
        echo dataset-${seq}_512_16
    elif [[ $1 == "uzh_fpv" ]]; then
        if [[ "${uzh_fpv_seqs_w_gt[*]}" =~ $(echo "\<${seq}\>") ]]; then
            echo ${seq}_snapdragon_with_gt
        elif [[ "${uzh_fpv_seqs_no_gt[*]}" =~ $(echo "\<${seq}\>") ]]; then
            echo ${seq}_snapdragon
        else
            echo "full_seq_name_error_uzh_fpv"
        fi
    else
        echo "full_seq_name_error"
    fi
}

cd ${dump_dir}

if [[ $5 == "--dense" ]]; then
    dense_opt="dense:=true"
else
    dense_opt="dense:=false"
fi

if [[ ${dataset} == "euroc" ]]; then
    seqs_to_run=(${euroc_seqs[@]})
    dataset_dir=${euroc_dataset_dir}
elif [[ ${dataset} == "tumvio" ]]; then
    seqs_to_run=(${tumvio_seqs[@]})
    dataset_dir=${tumvio_dataset_dir}
elif [[ ${dataset} == "uzh_fpv" ]]; then
    seqs_to_run=("${uzh_fpv_seqs_w_gt[@]}" "${uzh_fpv_seqs_no_gt[@]}")
    dataset_dir=${uzh_fpv_dataset_dir}
else
    echo "dataset not valid"
    exit 1
fi

if [[ ${seqs} != "all" ]]; then
    unset seqs_to_run
    declare -a seqs_to_run
    seqs_to_run[0]=${seqs}
fi

echo "========= Running sequences ${seqs_to_run[@]} ========="

echo "========= estimator ${estimator} ========="
echo "========= dataset ${dataset} ========="
echo "========= seqs ${seqs} ========="
echo "========= dense_opt ${dense_opt} ========="

if [[ ${estimator} == "vins_mono" ]]; then
    for i in "${!seqs_to_run[@]}"
    do
        seq=${seqs_to_run[$i]}
        if [[ ${dataset} == "euroc" ]]; then
            launch_file="euroc.launch"
        elif [[ ${dataset} == "tumvio" ]]; then
            launch_file="tum.launch"
        elif [[ ${dataset} == "uzh_fpv" ]]; then
            launch_file="uzh_fpv.launch"
            if [[ ${seq} =~ "indoor_45" ]]; then
                config_opt="${okvis_uzh_fpv_configs_dir}/uzh_fpv_indoor_45_config.yaml"
            elif [[ ${seq} =~ "indoor_forward" ]]; then
                config_opt="${okvis_uzh_fpv_configs_dir}/uzh_fpv_indoor_forward_config.yaml"
            elif [[ ${seq} =~ "outdoor_45" ]]; then
                config_opt="${okvis_uzh_fpv_configs_dir}/uzh_fpv_outdoor_45_config.yaml"
            elif [[ ${seq} =~ "outdoor_forward" ]]; then
                config_opt="${okvis_uzh_fpv_configs_dir}/uzh_fpv_outdoor_forward_config.yaml"
            else
                exit 1
            fi
        fi

        run_vins_mono_ros ${launch_file} ${dataset_dir}/bags/$(fullseqname ${dataset} ${seq}).bag ${dense_opt} config_path:=${config_opt}
        mv ${dump_dir}/vins_result_no_loop.csv ${results_dir}/${seq}_vins_result_no_loop.csv
        python ${vinsmono2tum_script} ${results_dir}/${seq}_vins_result_no_loop.csv
        mv ${results_dir}/okvis_estimator_output.tum ${results_dir}/${seq}_okvis_estimator_output.tum
    done
elif [[ ${estimator} == "okvis" ]]; then
    for i in "${!seqs_to_run[@]}"
    do
        seq=${seqs_to_run[$i]}
        echo $(fullseqname ${dataset} ${seq})
        if [[ ${dataset} == "uzh_fpv" ]]; then
            launch_file="okvis_node_uzh_fpv.launch"
            if [[ ${seq} =~ "indoor_45" ]]; then
                config_opt="${okvis_uzh_fpv_configs_dir}/uzh_fpv_indoor_45.yaml"
            elif [[ ${seq} =~ "indoor_forward" ]]; then
                config_opt="${okvis_uzh_fpv_configs_dir}/uzh_fpv_indoor_forward.yaml"
            elif [[ ${seq} =~ "outdoor_45" ]]; then
                config_opt="${okvis_uzh_fpv_configs_dir}/uzh_fpv_outdoor_45.yaml"
            elif [[ ${seq} =~ "outdoor_forward" ]]; then
                config_opt="${okvis_uzh_fpv_configs_dir}/uzh_fpv_outdoor_forward.yaml"
            else
                exit 1
            fi
            run_okvis_ros ${launch_file} ${dataset_dir}/bags/$(fullseqname ${dataset} ${seq}).bag config_path:=${config_opt}
        else
            if [[ ${dataset} == "euroc" ]]; then
                config_yaml="${okvis_uzh_fpv_configs_dir}/config_fpga_p2_euroc.yaml"
            elif [[ ${dataset} == "tumvio" ]]; then
                config_yaml="${okvis_uzh_fpv_configs_dir}/config_okvis_50_20.yaml"
            fi
            rosrun okvis_ros okvis_node_synchronous_from_file ${config_yaml} ${dataset_dir}/${seq}/mav0
        fi
        mv ${dump_dir}/okvis_estimator_output.csv ${results_dir}/${seq}_okvis_estimator_output.csv
        python ${okvis2tum_script} ${results_dir}/${seq}_okvis_estimator_output.csv
        mv ${results_dir}/okvis_estimator_output.tum ${results_dir}/${seq}_okvis_estimator_output.tum
    done
elif [[ ${estimator} == "open_vins" ]]; then
    if [[ ${dataset} == "euroc" ]]; then
        launch_file="pgeneva_eth.launch"
    elif [[ ${dataset} == "tumvio" ]]; then
        launch_file="pgeneva_tum.launch"
    elif [[ ${dataset} == "uzh_fpv" ]]; then
        launch_file="uzh_fpv.launch"
    fi
else
    echo "Bad estimator"
    exit 1
fi