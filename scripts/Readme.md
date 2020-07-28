## Dependencies
1. Python 3, probably nice to make a virtulenv for this
1. `sudo apt-get install unzip`
1. `sudo apt-get install jq`
1. numpy (`pip3 install numpy`)
1. matplotlib (`pip3 install matplotlib`)
1. evo (`pip3 install evo`)
1. need to have ROS kinetic, ubuntu 16.04 (other versions should work)
1. You need your vins_fusion compiled, with catkin workspace sourced
1. vins_fusion config: change:
   - `output_path: "/home/your_username/Dev/dump/"`
   - `pose_graph_save_path: "/home/your_username/Dev/dump/"`

## Directory Structure
```bash
mkdir -p ~/Dev/dump  # where results are dumped

# your catkin ws must be at ~/Dev/catkin_ws, but you can just symlink I thikn
# This might not be necessary for just vins_fusion
ln -s your_catkin_ws ~/Dev/catkin_ws

mkdir -p ~/Dev/EUROC # where EuRoC Data is
```

#### EuRoC Directory Structure
```
~/Dev/EUROC
    ├── bags
    │   ├── MH_01_easy.bag
    │   ├── MH_02_easy.bag
    │   ├── MH_03_medium.bag
    │   ├── MH_04_difficult.bag
    │   ├── MH_05_difficult.bag
    │   ├── V1_01_easy.bag
    │   ├── V1_02_medium.bag
    │   ├── V1_03_difficult.bag
    │   ├── V2_01_easy.bag
    │   ├── V2_02_medium.bag
    │   └── V2_03_difficult.bag
    ├── MH_01_easy
    │   └── mav0
    │       ├── body.yaml
    │       ├── cam0
    │       ├── cam1
    │       ├── imu0
    │       ├── leica0
    │       └── state_groundtruth_estimate0
    ├── MH_02_easy
    │   └── mav0
    │       ├── body.yaml
    │       ├── cam0
    │       ├── cam1
    │       ├── imu0
    │       ├── leica0
    │       └── state_groundtruth_estimate0
... ...
```

## How to run

```bash
cd scripts
```

```bash
# === RUN ONE TIME ===
# position parameters: 
# 1. Chosen Method: vins_fusion / vins_mono / euroc
# 2. Chosen Dataset: euroc / tumvio / uzh_fpv
# 3. sequence name or all of them: i.e. MH_03_medium / V1_01_easy / all
# 4. some path to dump all the final outputs, you must have ~/Dev/dump for intermediary outputs
./run_vio.sh vins_fusion euroc MH_03_medium /home/cs4li/Dev/dump/test

# === Evaluate ONE TIME ===
# positional parameters:
# 1. Chosen Dataset
# 2. Where you've dumped the outputs
# 3. sequence name or all of them
./calc_error.sh euroc /home/cs4li/Dev/dump/test MH_03_medium


# === RUN MULTIPLE TIMES ===
# 1. Chosen Method: vins_fusion / vins_mono / euroc
# 2. Chosen Dataset: euroc / tumvio / uzh_fpv
# 3. sequence name or all of them: i.e. MH_03_medium / V1_01_easy / all
# 4. some path to dump all the final outputs
./multirun_vio.sh vins_fusion euroc all 10 run_only /home/cs4li/Dev/dump/latest

# === Evaluate MULTIPLE TIMES === (just change the previous command from run_only to eval_only)
# 1. Chosen Method: vins_fusion / vins_mono / euroc
# 2. Chosen Dataset: euroc / tumvio / uzh_fpv
# 3. sequence name or all of them: i.e. MH_03_medium / V1_01_easy / all
# 4. Where you've dumped the outputs
./multirun_vio.sh vins_fusion euroc all 10 eval_only /home/cs4li/Dev/dump/latest

```