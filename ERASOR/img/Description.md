# ERASOR.Official

Official page of ["ERASOR: Egocentric Ratio of Pseudo Occupancy-based Dynamic Object Removal for Static 3D Point Cloud Map Building"](https://arxiv.org/abs/2103.04316), which is accepted by RA-L with ICRA'21 option.

We provide all contents including
* Source code of ERASOR
* All outputs of State-of-the-arts
* Visualization application
* Calculation code of Preservation Rate/Rejection Rate

So enjoy our codes! :)

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
sudo apt-get install ros-melodic-~~
sudo apt-get install ros-melodic-~~
sudo apt-get install ros-melodic-~~
sudo apt-get install ros-melodic-~~
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
- The downloading process might take five minutes or so. All rosbags requires 2.3G of storage space
```bash
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/00_4390_to_4532_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/01_150_to_251_w_interval_1_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/02_860_to_952_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/05_2350_to_2672_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/07_630_to_822_w_interval_2_node.bag
```

## How to Run

We will explain how to run our code on seq 05 of KITTI dataset as an example.

**Step 1. Build naive map**
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

- Error metrics on NYU Depth v2:

	| RGB     |  rms  |  rel  | delta1 | delta2 | delta3 |
	|-----------------------------|:-----:|:-----:|:-----:|:-----:|:-----:|
	| [Roy & Todorovic](http://web.engr.oregonstate.edu/~sinisa/research/publications/cvpr16_NRF.pdf) (_CVPR 2016_) | 0.744 | 0.187 |  - | - | - |
	| [Eigen & Fergus](http://cs.nyu.edu/~deigen/dnl/) (_ICCV 2015_)  | 0.641 | 0.158 | 76.9 | 95.0 | 98.8 |
	| [Laina et al](https://arxiv.org/pdf/1606.00373.pdf) (_3DV 2016_)            | 0.573 | **0.127** | **81.1** | 95.3 | 98.8 |
	| Ours-RGB             | **0.514** | 0.143 | 81.0 | **95.9** | **98.9** |

	| RGBd-#samples   |  rms  |  rel  | delta1 | delta2 | delta3 |
	|-----------------------------|:-----:|:-----:|:-----:|:-----:|:-----:|
	| [Liao et al](https://arxiv.org/abs/1611.02174) (_ICRA 2017_)-225 | 0.442 | 0.104 | 87.8 | 96.4 | 98.9 |
	| Ours-20 | 0.351 | 0.078 | 92.8 | 98.4 | 99.6 |
	| Ours-50 | 0.281 | 0.059 | 95.5 | 99.0 | 99.7 |
	| Ours-200| **0.230** | **0.044** | **97.1** | **99.4** | **99.8** |

	<img src="http://www.mit.edu/~fcma/images/ICRA18/acc_vs_samples_nyu.png" alt="photo not available" width="50%" height="50%">

- Error metrics on KITTI dataset:

	| RGB     |  rms  |  rel  | delta1 | delta2 | delta3 |
	|-----------------------------|:-----:|:-----:|:-----:|:-----:|:-----:|
	| [Make3D](http://papers.nips.cc/paper/5539-depth-map-prediction-from-a-single-image-using-a-multi-scale-deep-network.pdf) | 8.734 | 0.280 | 60.1 | 82.0 | 92.6 |
	| [Mancini et al](https://arxiv.org/pdf/1607.06349.pdf) (_IROS 2016_)  | 7.508 | - | 31.8 | 61.7 | 81.3 |
	| [Eigen et al](http://papers.nips.cc/paper/5539-depth-map-prediction-from-a-single-image-using-a-multi-scale-deep-network.pdf) (_NIPS 2014_)  | 7.156 | **0.190** | **69.2** | 89.9 | **96.7** |
	| Ours-RGB             | **6.266** | 0.208 | 59.1 | **90.0** | 96.2 |

	| RGBd-#samples   |  rms  |  rel  | delta1 | delta2 | delta3 |
	|-----------------------------|:-----:|:-----:|:-----:|:-----:|:-----:|
	| [Cadena et al](https://pdfs.semanticscholar.org/18d5/f0747a23706a344f1d15b032ea22795324fa.pdf) (_RSS 2016_)-650 | 7.14 | 0.179 | 70.9 | 88.8 | 95.6 |
	| Ours-50 | 4.884 | 0.109 | 87.1 | 95.2 | 97.9 |
	| [Liao et al](https://arxiv.org/abs/1611.02174) (_ICRA 2017_)-225 | 4.50 | 0.113 | 87.4 | 96.0 | 98.4 |
	| Ours-100 | 4.303 | 0.095 | 90.0 | 96.3 | 98.3 |
	| Ours-200 | 3.851 | 0.083 | 91.9 | 97.0 | 98.6 |
	| Ours-500| **3.378** | **0.073** | **93.5** | **97.6** | **98.9** |
	
	Note: our networks are trained on the KITTI odometry dataset, using only sparse labels from laser measurements.

## Citation 
If you use our code or method in your work, please consider citing the following:

	@article{Ma2017SparseToDense,
		title={Sparse-to-Dense: Depth Prediction from Sparse Depth Samples and a Single Image},
		author={Ma, Fangchang and Karaman, Sertac},
		booktitle={ICRA},
		year={2018}
	}
	@article{ma2018self,
		title={Self-supervised Sparse-to-Dense: Self-supervised Depth Completion from LiDAR and Monocular Camera},
		author={Ma, Fangchang and Cavalheiro, Guilherme Venturelli and Karaman, Sertac},
		journal={arXiv preprint arXiv:1807.00275},
		year={2018}
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



## Description

나의 부족으로 intensity에 label이 있는 상태임...

## How to use

1. bagfile 생성

```
$ python kitti2imnode.py -t None -r None -s 05 --kitti_type "odom_noimg"
```

* 382~384 줄의 interval과 처음 / 끝 지정해주어야 함!
* final_stamp가 미포함임 주의!!!

2. mapgen
```
$ roslaunch kitti_scdr mapgen.launch
```

코드 내에 주소 잘 지정해줘야 함!

3. dynamic removal

* 파라미터 1에서 지정해준 걸로 맞춰주어야 함! 

## 참고

* SCDR (ERASOR)는 현재 h + 1.73되어 있는데, ground filter는 LiDAR 기준이라서 헷깔!
* Dynamic objects class & ID 찾기

analysis.py 내부에 fetch_dynamic_objects_ids 함수 사용하면 됨!


## Transformation

* mapgen과 알고리즘
In KITTI - LiDAR가 위에 있기 때문에, 1.73 곱해주어야 함!

따라서 pc의 z에 +1.73해준 후에, tf_body2origin해주어야 함! (python에서 rosbag 만들 당시에 tf를 바닥기준으로 해서 주기 때문임)

* 20201221

KITTI data를 다시한번 볼 필요가 있음! 

* ERASOR 707 줄 주의해야 함!

근데 rosbag에는 

## 주의해야할 파라미터

threshold_h_percentage: The Larger, the more aggressive!!
