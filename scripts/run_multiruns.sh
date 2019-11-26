#!/usr/bin/env bash

./multirun_vio.bash okvis euroc 10 run_only ~/Dev/dump/EUROC_OKVIS "okvis on euroc 10 times"
./multirun_vio.bash vins_mono euroc 10 run_only ~/Dev/dump/EUROC_VINS_Mono "vins_mono on euroc 10 times"
./multirun_vio.bash okvis tumvio 10 run_only ~/Dev/dump/TUMVIO_OKVIS "OKVIS on tumvio 10 times"