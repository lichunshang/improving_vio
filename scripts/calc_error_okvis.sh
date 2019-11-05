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
    unzip -o ${dump_dir}/results.zip -d ${dump_dir}/
    jq ".rmse" ${dump_dir}/stats.json >> $4
}

if [[ $1 == "euroc" ]]; then
    euroc_dir="/home/cs4li/Dev/EUROC"
    euroc_results_dir="/home/cs4li/Dev/dump/Vanilla_dataset_runs/EUROC_OKVIS"

    echo "euroc_results_dir ${euroc_results_dir}"
    rm -rf ${euroc_results_dir}/stats_dump.txt
    rm -rf ${euroc_results_dir}/record.txt

    for i in "${!euroc_map_key_ordered[@]}"
    do
      key=${euroc_map_key_ordered[$i]}
      run_euroc ${euroc_dir}/${key}/mav0 ${euroc_results_dir}/${key}_okvis_estimator_output.csv ${euroc_results_dir}/${key} ${euroc_results_dir}/stats_dump.txt
    done
elif [[ $2 == "tumvio" ]]; then
    pass
fi

