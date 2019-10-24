#!/usr/bin/env bash
set -e
trap 'exit 130' INT

source_zips_dir=${1%/}
target_dump_dir=${2%/}

echo "source_zips_dir $source_zips_dir"
echo "target_dump_dir $target_dump_dir"


for f in "$source_zips_dir"/*
do
if [[ ${f} =~ \.zip$ ]]
then
    echo "found zip file $f"
    dest=${f##*/}
    dest=${target_dump_dir}/${dest%.zip}
    n_files=$(unzip -l ${f}  | wc -l)
    echo "Found $n_files items in zip, unzipping to $dest..."
    unzip ${f} -d ${dest} | pv -l -s ${n_files} > /dev/null
elif [[ ${f} =~ \.tar$ ]]
then
    echo "found tar file $f"
    dest=${f##*/}
    dest=${target_dump_dir}/${dest%.tar}
    echo "Found $n_files items in tar, unpack to $dest..."
    pv ${f} | tar xf - -C ${target_dump_dir}
fi
done