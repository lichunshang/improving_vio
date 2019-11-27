#!/usr/bin/env bash
set -e
trap 'exit 130' INT

echo "Arguments: <estimator> <dataset> <x_times> <mode> <save_dir> <statement>"
estimator=$1
dataset=$2
run_x_times=$3
mode=$4
save_dir=$5
statement=$6

echo "estimator ${estimator}"
echo "dataset ${dataset}"
echo "run_x_times ${run_x_times}"
echo "mode ${mode}"
echo "save_dir ${save_dir}"
echo "statement ${statement}"


this_script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

mkdir -p ${save_dir}

echo ${statement} >> ${save_dir}/statement.txt

if [[ ${mode} == "run_only" ]] || [[ ${mode} == "both" ]]; then
    for i in $(seq 1 ${run_x_times}); do
        mkdir -p ${save_dir}/run_${i}
        ${this_script_dir}/run_vio.sh ${estimator} ${dataset} all ${save_dir}/run_${i}
    done
fi

if [[ ${mode} == "eval_only" ]] || [[ ${mode} == "both" ]]; then
    for i in $(seq 1 ${run_x_times}); do
        ${this_script_dir}/calc_error.sh ${dataset} ${save_dir}/run_${i} all
    done
    python ${this_script_dir}/collect_errors.py ${dataset} "multi_run" ${save_dir} ${save_dir}/record.txt
fi