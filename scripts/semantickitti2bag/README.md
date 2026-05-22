Date: 21.11.12

# Data Generation for ERASOR (Ours)

```
python kitti2node.py -t None -r None -s 00 --kitti_type "odom_noimg" --init_stamp 0 --end_stamp 4542
```

Set the frame range at line 382.

# Rosrun

```
rosbag play
rosrun scdr kitti_mapgen
```

# Remember!!!

**For the lastmile project, `poses.txt` is replaced with the GT poses!!!**

Adding `[0]` is important because the C++ side drops the first data sample (see `kitti2imnode`, line 383).

`kitti2offline`: the workspace used while writing the SCDR (ERASOR) paper.

* The SuMa poses must be placed at `/home/shapelim/hdd2/kitti_semantic/dataset/sequences/00/poses.txt`.

* The label field is originally `uint32`, but we store it inside `intensity` by casting to `float32`. On the C++ side it needs to be parsed back as `uint32_t`.

```
np.uint32  <-> uint32_t in c++
np.float32 <-> float    in c++
```

Originally,

```python
CAM2LIDAR = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                      [-6.481465826011e-03,  8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02],
                      [ 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                      [ 0, 0, 0, 1]])
```

However, because we use the height value, we fine-tune this transform as follows:

```python
CAM2BASE = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                     [-6.481465826011e-03,  8.051860151134e-03, -9.999466081774e-01,  1.65],
                     [ 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                     [ 0, 0, 0, 1]])
```
