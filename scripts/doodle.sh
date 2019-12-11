#!/usr/bin/env bash

#multirun_vio.sh okvis euroc 10 run_only ~/Dev/dump/EUROC_OKVIS "okvis on euroc 10 times"
#multirun_vio.sh vins_mono euroc 10 run_only ~/Dev/dump/EUROC_VINS_Mono "vins_mono on euroc 10 times"
#multirun_vio.sh okvis tumvio 10 run_only ~/Dev/dump/TUMVIO_OKVIS "OKVIS on tumvio 10 times"

#./multirun_vio.sh vins_mono euroc all 10 run_only /home/cs4li/Dev/dump/EUROC_VINS_DENSE_LK_refine_keep_LK_fails_stacked "EUROC Dense with LK refine, and if LK fails use Dense, process every image stack flow" --dense
#./multirun_vio.sh vins_mono tumvio all 10 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_DENSE_LK_refine_keep_LK_fails "TUMVIO Dense with LK refine, and if LK fails use Dense" --dense

#./multirun_vio.sh vins_mono euroc V2_03_difficult 10 run_only /home/cs4li/Dev/dump/test_doodle "test" --dense
#./multirun_vio.sh vins_mono euroc V2_01_easy 5 run_only /home/cs4li/Dev/dump/test_doodle "test" --dense
#./multirun_vio.sh vins_mono euroc MH_01_easy 5 run_only /home/cs4li/Dev/dump/test_doodle "test" --dense
#./multirun_vio.sh vins_mono euroc MH_03_medium 5 run_only /home/cs4li/Dev/dump/test_doodle "test" --dense
#./multirun_vio.sh vins_mono euroc MH_05_difficult 5 run_only /home/cs4li/Dev/dump/test_doodle "test" --dense
#./multirun_vio.sh vins_mono euroc MH_04_difficult 5 run_only /home/cs4li/Dev/dump/test_doodle "test" --dense

#./multirun_vio.sh vins_mono euroc MH_02_easy 5 run_only /home/cs4li/Dev/dump/EUROC_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono euroc V1_01_easy 5 run_only /home/cs4li/Dev/dump/EUROC_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono euroc V1_02_medium 5 run_only /home/cs4li/Dev/dump/EUROC_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono euroc V1_03_difficult 5 run_only /home/cs4li/Dev/dump/EUROC_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono euroc V2_02_medium 5 run_only /home/cs4li/Dev/dump/EUROC_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#
#./multirun_vio.sh vins_mono tumvio corridor1 5 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono tumvio corridor3 5 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#
#./multirun_vio.sh vins_mono tumvio room1 5 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono tumvio room2 5 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono tumvio room3 5 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono tumvio room4 5 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono tumvio room5 5 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense
#./multirun_vio.sh vins_mono tumvio room6 5 run_only /home/cs4li/Dev/dump/TUMVIO_VINS_Mono_DENSE_LK_refine_keep_LK_fails_stacked_5px "run" --dense

#./multirun_vio.sh vins_mono tumvio room1 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense
#./multirun_vio.sh vins_mono tumvio room2 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense
#./multirun_vio.sh vins_mono tumvio room3 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense
#./multirun_vio.sh vins_mono tumvio room4 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense
#./multirun_vio.sh vins_mono tumvio room5 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense
#./multirun_vio.sh vins_mono tumvio room6 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense
#
#./multirun_vio.sh vins_mono tumvio corridor2 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense
#./multirun_vio.sh vins_mono tumvio corridor4 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense

./multirun_vio.sh vins_mono euroc all 5 run_only /home/cs4li/Dev/dump/test_doodle_euroc  "run" --dense
./multirun_vio.sh vins_mono tumvio all 5 run_only /home/cs4li/Dev/dump/test_doodle_tumvio  "run" --dense