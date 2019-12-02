#!/usr/bin/env bash
set -e
#set -x
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

dataset=$1
results_dir=$2
seqs=$3

echo "dataset ${dataset}"
echo "results_dir ${results_dir}"
echo "seqs ${seqs}"

this_script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

stats_dump_file=${results_dir}/stats_dump.txt
tumvio_mocap2tum_script="${this_script_dir}/tumvio_mocap2tum.py"
uzh_fpv_gt2tum_script="${this_script_dir}/uzh_fpv_gt2tum.py"
euroc_dataset_dir="/home/cs4li/Dev/EUROC"
tumvio_dataset_dir="/home/cs4li/Dev/TUMVIO"
uzh_fpv_dataset_dir="/home/cs4li/Dev/UZH_FPV"

rm -rf ${stats_dump_file}*


function fullseqname() {
    seq=$2
    if [[ $1 == "euroc" ]]; then
        echo ${seq}
    elif [[ $1 == "tumvio" ]]; then
        echo dataset-${seq}_512_16
    elif [[ $1 == "uzh_fpv" ]]; then
        echo ${seq}_snapdragon_with_gt
    else
        echo "full seq name error"
    fi
}

cd ${results_dir}

if [[ ${dataset} == "euroc" ]]; then
    seqs_to_run=(${euroc_seqs[@]})
    dataset_dir=${euroc_dataset_dir}
elif [[ ${dataset} == "tumvio" ]]; then
    seqs_to_run=(${tumvio_seqs[@]})
    dataset_dir=${tumvio_dataset_dir}
elif [[ ${dataset} == "uzh_fpv" ]]; then
    seqs_to_run=(${uzh_fpv_seqs_w_gt[@]})
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
echo "========= dataset ${dataset} ========="
echo "========= seqs ${seqs} ========="

for i in "${!seqs_to_run[@]}"
do
    seq=${seqs_to_run[$i]}
    traj_path=$(find ${results_dir} -maxdepth 1 -name ${seq}_*.tum)
    if [[ -z ${traj_path} ]]; then
        echo "*** ${seq} not found, skipped"
        continue
    fi
    seq_dir=${dataset_dir}/$(fullseqname ${dataset} ${seq})
    seq_results_dir=${results_dir}/${seq}

    echo "seq ${seq}"
    echo "seq_dir ${seq_dir}"
    echo "traj_path ${traj_path}"

    mkdir -p "${seq_results_dir}"
    cp ${traj_path} ${seq_results_dir}/est.tum

    echo "seq_dir ${seq_dir}"

    if [[ ${dataset} == "euroc" ]]; then
        evo_traj euroc ${seq_dir}/mav0/state_groundtruth_estimate0/data.csv --save_as_tum
        mv data.tum ${seq_results_dir}/gt.tum
    elif [[ ${dataset} == "tumvio" ]]; then
        python ${tumvio_mocap2tum_script} ${seq_dir}/mav0/mocap0/data.csv
        mv ${seq_dir}/mav0/mocap0/data_tum.csv ${seq_results_dir}/gt.tum
    elif [[ ${dataset} == "uzh_fpv" ]]; then
        python ${uzh_fpv_gt2tum_script} ${seq_dir}/groundtruth.txt
        mv ${seq_dir}/groundtruth_tum.txt ${seq_results_dir}/gt.tum
    fi

    echo "*** make some plots..."
    evo_traj tum ${seq_results_dir}/gt.tum --save_plot ${seq_results_dir}/plot_gt.pdf --plot_mode xyz --no_warnings
    evo_traj tum ${seq_results_dir}/est.tum --save_plot ${seq_results_dir}/plot_est.pdf --plot_mode xyz --no_warnings
    evo_traj tum ${seq_results_dir}/est.tum --ref ${seq_results_dir}/gt.tum --save_plot ${seq_results_dir}/plot_overlay.pdf --plot_mode xyz --align --no_warnings

    echo '#!/usr/bin/env bash' > ${seq_results_dir}/view_overlay.sh
    echo 'DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )' >> ${seq_results_dir}/view_overlay.sh
    echo 'evo_traj tum ${DIR}/est.tum --ref ${DIR}/gt.tum --save_plot ${seq_results_dir}/plot_overlay.pdf --plot --align' >> ${seq_results_dir}/view_overlay.sh
    chmod +x ${seq_results_dir}/view_overlay.sh

    echo "****evo ape evaluation of trans part..."
    evo_ape tum ${seq_results_dir}/gt.tum ${seq_results_dir}/est.tum -r trans_part --align \
        --no_warnings --logfile ${results_dir}/evo_record.txt --save_results  ${seq_results_dir}/results.zip \
        --plot_mode xyz --save_plot ${seq_results_dir}/plot.pdf \

    unzip -o ${seq_results_dir}/results.zip -d ${seq_results_dir}/evo_results
    rm -rf ${seq_results_dir}/results.zip

    echo "***save some stats"
    python ${this_script_dir}/evo_traj_full_eval.py ${seq_results_dir}/gt.tum ${seq_results_dir}/est.tum ${seq_results_dir} ${results_dir}/my_record.txt

    trans_rmse=$(jq ".ape_trans" ${seq_results_dir}/stats.json)
    rot_rmse=$(jq ".ape_rot_deg" ${seq_results_dir}/stats.json)
    echo -n "${trans_rmse} ${rot_rmse} " >> ${stats_dump_file}
done

# collect the rpe stats over the entire run
python ${this_script_dir}/collect_errors.py ${dataset} "single_run" ${results_dir} ${results_dir}/my_record.txt
