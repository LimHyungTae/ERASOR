# ERASOR.Official

Official page of [*"ERASOR: Egocentric Ratio of Pseudo Occupancy-based Dynamic Object Removal for Static 3D Point Cloud Map Building"*](https://arxiv.org/abs/2103.04316), which is accepted by RA-L with ICRA'21 option [[Demo Video](https://www.youtube.com/watch?v=Nx27ZO8afm0)].

![00_knn](img/fig_overview.pdf)

We provide all contents including

-[x] Source code of ERASOR
-[x] All outputs of State-of-the-arts
-[x] Visualization application 
-[x] Calculation code of Preservation Rate/Rejection Rate

So enjoy our codes! :)

(***PLEASE REMIND THAT THIS REPO IS IN PROGRESS!! May be it be will completed by ~6/06***)

Contact: Hyungtae Lim (shapelim@kaist.ac.kr)
Advisor: Hyun Myung (hmyung@kaist.ac.kr)

## Contents
0. [Reliability](#Reliability)
0. [Requirements](#requirements)
0. [How to run ERASOR](#How to Run)
0. [Calculate PR/RR](#Calculate PR/RR)
0. [Trained Models](#trained-models)
0. [Benchmark](#benchmark)
0. [Citation](#citation)

### Reliability
The code is tested successfully at
* Linux 18.04 LTS
* ROS Melodic

## Requirements

### ROS Setting
- Install [ROS](http://torch.ch/docs/getting-started.html) on a machine. 
- Also, [jsk-visualization](https://github.com/jsk-ros-pkg/jsk_visualization) is required to run our code.

```bash
sudo apt-get install ros-melodic-jsk-recognition
sudo apt-get install ros-melodic-jsk-common-msgs
sudo apt-get install ros-melodic-rviz-plugins
```
 
### Python Setting
- Our metric calculation code is implemented by python2.7
- To run the python code, following pakages are necessary: [pypcd](https://github.com/dimatura/pypcd), [tqdm](https://github.com/tqdm/tqdm), [scikit-learn](https://scikit-learn.org/stable/), and [tabulate](https://pyneng.readthedocs.io/en/latest/book/12_useful_modules/tabulate.html)
```bash
pip install pypcd
pip install tqdm	
pip install scikit-learn
pip install tabulate
```
 
### Prepared dataset

- Download the preprocessed KITTI data encoded into rosbag.
- The downloading process might take five minutes or so. All rosbags requires total 2.3G of storage space
```bash
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/00_4390_to_4530_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/01_150_to_250_w_interval_1_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/02_860_to_950_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/05_2350_to_2670_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/07_630_to_820_w_interval_2_node.bag
```
- Please note that the rosbag consists of 
```
# 252: "moving-car"
# 253: "moving-bicyclist"
# 254: "moving-person"
# 255: "moving-motorcyclist"
# 256: "moving-on-rails"
# 257: "moving-bus"
# 258: "moving-truck"
# 259: "moving-other-vehicle"
```


## How to Run

We will explain how to run our code on seq 05 of KITTI dataset as an example.

**Step 1. Build naive map**

![kittimapgen](img/kittimapgen.gif)
* Set the following parameters in `launch/mapgen.launch`.
	* `target_rosbag`: The name of target rosbag, e.g. `05_2350_to_2670_w_interval_2_node.bag`
	* `save_path`: The path where the naively accumulated map is saved.
* Launch mapgen.launch and play corresponding rosbag on the other bash as follows:
```bash
roscore # (Optional)
roslaunch erasor mapgen.launch
rosbag play 05_2350_to_2670_w_interval_2_node.bag
```
* Then, dense map and voxelized map are auto-saved at the save path. Note that dense map is used to fill corresponding labels (**To be explained**). Voxelized map will be filled by step 2 Voxelized map will be filled by step 2.

**Step 2. Run ERASOR**
![erasor](img/kitti05.gif)
* Set the following parameters in `config/seq_05.yaml`.
	* `initial_map_path`: The path of naively accumulated map
	* `save_path`: The path where the filtered static map is saved.
  
* Run the following command for each bash.
```bash
roscore # (Optional)
roslaunch erasor run_erasor.launch target_seq:="05"
rosbag play 05_2350_to_2672_w_interval_2_node.bag
```
* **IMPORTANT:** After finishing running ERASOR, run follow command to save the filtered map as pcd file on another bash.
* "0.2" denotes voxelization size.
```bash
rostopic pub /saveflag std_msgs/Float32 "data: 0.2"
```

* The results will be saved under the `save_path` folder, i.e. `$save_path$/05_result.pcd`.

**Step 3. Calculate PR/RR**

```bash
python analysis.py
```

## Benchmark

- Error metrics are a little bit different from those in the paper:
  
  | Seq.    |  PR  |  RR|
  |-----------------------------|:-----:|:-----:|
  | 00  | 91.72 | 97.00 |
  | 01  | 91.93 | 94.63 |
  | 02  | 81.08 | 99.11 |
  | 05  | 86.98 | 97.88 |
  | 07  | 92.00 | 98.33 |

## Citation 
If you use our code or method in your work, please consider citing the following:

	@article{lim2021erasor,
    title={ERASOR: Egocentric Ratio of Pseudo Occupancy-Based Dynamic Object Removal for Static 3D Point Cloud Map Building},
    author={Lim, Hyungtae and Hwang, Sungwon and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    volume={6},
    number={2},
    pages={2272--2279},
    year={2021},
    publisher={IEEE}
    }

## How to visualize results

0. Build the repository: `caktin_make erasor` (or `catkin build erasor` if you use [catkin tools](https://catkin-tools.readthedocs.io/en/latest/))

1. Set parameters in `config/viz_params.yaml` correctly, i.e. your absolute directory and target sequence. 

2. `roslaunch erasor compare_results.launch`

## Results

### [Sequence 00: 4,390~4,530](imgs/00)

### [Sequence 01: 150~250](imgs/01)

### [Sequence 02: 860~950](imgs/02)

### [Sequence 05: 2,350~2670](imgs/05)

### [Sequence 07: 630~820](imgs/07)



