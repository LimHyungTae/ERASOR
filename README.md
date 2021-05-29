ERASOR on KITTI dataset v1
=======================

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
