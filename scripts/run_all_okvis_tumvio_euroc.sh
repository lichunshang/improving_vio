#!/usr/bin/env bash
set -e
trap 'exit 130' INT

a="okvis_ros"
b="okvis_node_synchronous_from_file"

if [[ $2 == "--from_scratch" ]]
then
    echo "*** Running all from scratch"
    from_scratch=1
else
    echo "*** Only filling in "
    from_scratch=0
fi

if [[ $1 == "tumvio" ]]; then
tum_dir="/home/cs4li/Dev/TUMVIO"
tumvio_yaml="/home/cs4li/Dev/catkin_ws/src/okvis_ros/okvis/config/config_okvis_50_20.yaml"
for seq in ${tum_dir}/*
do
echo "*** found $seq"
if [[ ! -f $seq/mav0/okvis_estimator_output.csv ]] || [[ ${from_scratch} == 1 ]]; then
    echo "*** Executing OKVIS"
    rosrun ${a} ${b} ${tumvio_yaml} ${seq}/mav0
else
    echo "*** Found okvis output, skipped"
fi
done
fi

if [[ $1 == "euroc" ]]; then
euroc_dir="/home/cs4li/Dev/EUROC"
euroc_yaml="/home/cs4li/Dev/catkin_ws/src/okvis_ros/okvis/config/config_fpga_p2_euroc.yaml"
for seq in ${euroc_dir}/*
do
echo "*** found $seq"
if [[ ! -f $seq/mav0/okvis_estimator_output.csv ]] || [[ ${from_scratch} == 1 ]]; then
    echo "*** Executing OKVIS"
    rosrun ${a} ${b} ${euroc_yaml} ${seq}/mav0
else
    echo "*** Found okvis output, skipped"
fi
done
fi