Date: 21.11.12

# Data Generation for ERASOR (Ours)

```
python kitti2node.py -t None -r None -s 00 --kitti_type "odom_noimg" --init_stamp 0 --end_stamp 4542
```

Set range in 382 line.

# Rosrun

```
rosbag play 
rosrun scdr kitti_mapgen
```
# Remember!!!

**For the lastmile project, the poses.txt is changed to gt poses!!!**

Add [0] is important since the cpp drop the first data. (kitti2imnode on 383 line)
    
kitti2offline : SCDR 논문 쓰기 위해서 하는 workspace

* SuMa의 pose를 /home/shapelim/hdd2/kitti_semantic/dataset/sequences/00/poses.txt에 여기 넣어둬야 함!

* label은 원래 uint32인데, intensity에 넣기 위해서 float 32로 변경해서 넣어줌. C++ 에서 uint32로 되돌려서 파싱 필요

np.uint32 <-> uint32_t in c++
np.float32 <-> float in c++

Originally, 

CAM2LIDAR = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                      [- 6.481465826011e-03, 8.051860151134e-03, - 9.999466081774e-01, - 7.337429464231e-02],
                      [9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                     [0, 0, 0, 1]])
                     
하지만 우리는 높이를 쓰기 때문에, 다음과 같이 fine tuning함                     
CAM2BASE = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                      [- 6.481465826011e-03, 8.051860151134e-03, - 9.999466081774e-01, 1.65],
                      [9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                      [0, 0, 0, 1]])
