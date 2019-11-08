#!/usr/bin/env bash
set -e
trap 'exit 130' INT

declare -a euroc_map_key_ordered
euroc_map_key_ordered[0]="MH_01_easy"
euroc_map_key_ordered[1]="MH_02_easy"
euroc_map_key_ordered[2]="MH_03_medium"
euroc_map_key_ordered[3]="MH_04_difficult"
euroc_map_key_ordered[4]="MH_05_difficult"

euroc_map_key_ordered[5]="V1_01_easy"
euroc_map_key_ordered[6]="V1_02_medium"
euroc_map_key_ordered[7]="V1_03_difficult"

euroc_map_key_ordered[8]="V2_01_easy"
euroc_map_key_ordered[9]="V2_02_medium"
euroc_map_key_ordered[10]="V2_03_difficult"

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


function run_euroc() {
    seq_dir=$1
    traj_path=$2
    dump_dir=$3
    traj_dir=$(dirname "${traj_path}")
    this_script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

    mkdir -p "${dump_dir}"

    echo "seq_dir ${seq_dir}"
    echo "this_script_dir ${this_script_dir}"

    echo "****Converting okvis estimator output to tum format..."
    python ${this_script_dir}/okvis_output2tum.py "${traj_path}"
    mv ${traj_dir}/okvis_estimator_output.tum ${dump_dir}

    echo "****evo evaluation..."
    evo_ape euroc "${seq_dir}/state_groundtruth_estimate0/data.csv" "${dump_dir}/okvis_estimator_output.tum" \
        --align --logfile ${traj_dir}/record.txt \
        --save_results  ${dump_dir}/results.zip \
        --plot_mode xyz --save_plot ${dump_dir}/plot \
        --no_warnings
    evo_traj tum "${dump_dir}/okvis_estimator_output.tum" --save_plot ${dump_dir}/plot_xyz --plot_mode xyz
    unzip -o ${dump_dir}/results.zip -d ${dump_dir}/
    jq ".rmse" ${dump_dir}/stats.json >> $4
}

function run_tumvio() {
    seq_dir=$1
    traj_path=$2
    dump_dir=$3
    traj_dir=$(dirname "${traj_path}")
    this_script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

    mkdir -p "${dump_dir}"

    echo "seq_dir ${seq_dir}"
    echo "this_script_dir ${this_script_dir}"

    echo "****Converting okvis estimator output to tum format..."
    python ${this_script_dir}/okvis_output2tum.py "${traj_path}"
    mv ${traj_dir}/okvis_estimator_output.tum ${dump_dir}

    echo "****Converting tumvio gt to tum format..."
    python ${this_script_dir}/tumvio_mocap2tum.py ${seq_dir}/mocap0/data.csv

    echo "****evo evaluation..."
    evo_ape tum "${seq_dir}/mocap0/data_tum.csv" "${dump_dir}/okvis_estimator_output.tum" \
        --align --logfile ${traj_dir}/record.txt \
        --save_results  ${dump_dir}/results.zip \
        --plot_mode xyz --save_plot ${dump_dir}/plot \
        --no_warnings
    evo_traj tum "${dump_dir}/okvis_estimator_output.tum" --save_plot ${dump_dir}/plot_xyz --plot_mode xyz --no_warnings
    unzip -o ${dump_dir}/results.zip -d ${dump_dir}/
    jq ".rmse" ${dump_dir}/stats.json >> $4
}

results_dir=$2
echo "results_dir ${results_dir}"
rm -rf ${results_dir}/stats_dump.txt
rm -rf ${results_dir}/record.txt


if [[ $1 == "euroc" ]]; then
    euroc_dir="/home/cs4li/Dev/EUROC"
    for i in "${!euroc_map_key_ordered[@]}"
    do
      key=${euroc_map_key_ordered[$i]}
      run_euroc ${euroc_dir}/${key}/mav0 ${results_dir}/${key}_okvis_estimator_output.csv ${results_dir}/${key} ${results_dir}/stats_dump.txt
    done
elif [[ $1 == "tumvio" ]]; then
    tumvio_dir="/home/cs4li/Dev/TUMVIO"
    for i in "${!tumvio_seqs[@]}"
    do
      key=${tumvio_seqs[$i]}
      run_tumvio ${tumvio_dir}/dataset-${key}_512_16/mav0 ${results_dir}/dataset-${key}_512_16_okvis_estimator_output.csv ${results_dir}/${key} ${results_dir}/stats_dump.txt
    done
fi

