#!/usr/bin/env bash
set -e
trap 'exit 130' INT

declare -A map=()
map["MH_01"]="MH_01_easy"
map["MH_02"]="MH_02_easy"
map["MH_03"]="MH_03_medium"
map["MH_04"]="MH_04_difficult"
map["MH_05"]="MH_05_difficult"
map["V1_01"]="V1_01_easy"
map["V1_02"]="V1_02_medium"
map["V1_03"]="V1_03_difficult"
map["V2_01"]="V2_01_easy"
map["V2_02"]="V2_02_medium"
map["V2_03"]="V2_03_difficult"

declare -a map_key_ordered
map_key_ordered[0]="MH_01"
map_key_ordered[1]="MH_02"
map_key_ordered[2]="MH_03"
map_key_ordered[3]="MH_04"
map_key_ordered[4]="MH_05"

map_key_ordered[5]="V1_01"
map_key_ordered[6]="V1_02"
map_key_ordered[7]="V1_03"

map_key_ordered[8]="V2_01"
map_key_ordered[9]="V2_02"
map_key_ordered[10]="V2_03"

function run() {
    seq_dir=$1
    traj_path=$2
    dump_dir=$3
    traj_dir=$(dirname "${traj_path}")
    this_script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

    mkdir -p "${dump_dir}"

    echo "seq_dir ${seq_dir}"
    echo "this_script_dir ${this_script_dir}"

    echo "****Converting vins estimator output to tum format..."
    python ${this_script_dir}/vinsmono_output2tum.py "${traj_path}"
    mv ${traj_dir}/vinsmono_output.tum ${dump_dir}

    echo "****evo evaluation..."
    evo_ape euroc "${seq_dir}/state_groundtruth_estimate0/data.csv" "${dump_dir}/vinsmono_output.tum" \
        --align --logfile ${traj_dir}/record.txt \
        --save_results  ${dump_dir}/results.zip \
        --plot_mode xyz --save_plot ${dump_dir}/plot \
        --no_warnings
    unzip -o ${dump_dir}/results.zip -d ${dump_dir}/
    jq ".rmse" ${dump_dir}/stats.json >> $4
}

euroc_dir="/home/cs4li/Dev/EUROC"
if [[ $1 == "vanilla" ]]; then
    euroc_results_dir="/home/cs4li/Dev/dump/Vanilla_dataset_runs/EUROC_VINS_MONO"
elif [[ $1 == "dense" ]]; then
    euroc_results_dir="/home/cs4li/Dev/dump/Dense_dataset_runs/EUROC_VINS_MONO"
else
    exit 1
fi

echo "euroc_results_dir ${euroc_results_dir}"
rm -rf ${euroc_results_dir}/stats_dump.txt
rm -rf ${euroc_results_dir}/record.txt

for i in "${!map_key_ordered[@]}"
do
  key=${map_key_ordered[$i]}
  run ${euroc_dir}/${map[${key}]}/mav0 ${euroc_results_dir}/${key}_vins_result_no_loop.csv ${euroc_results_dir}/${key} ${euroc_results_dir}/stats_dump.txt
done