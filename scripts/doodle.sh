#!/usr/bin/env bash

multirun_vio.sh okvis euroc 10 run_only ~/Dev/dump/EUROC_OKVIS "okvis on euroc 10 times"
multirun_vio.sh vins_mono euroc 10 run_only ~/Dev/dump/EUROC_VINS_Mono "vins_mono on euroc 10 times"
multirun_vio.sh okvis tumvio 10 run_only ~/Dev/dump/TUMVIO_OKVIS "OKVIS on tumvio 10 times"