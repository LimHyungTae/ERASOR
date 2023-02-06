#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os

import pykitti
from erasor.msg import node

import tf
import os
from geometry_msgs.msg import Pose
import cv2
import rospy
import rosbag
import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import argparse

def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = tf.transformations.quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        bag.write(topic, imu, t=imu.header.stamp)


def save_dynamic_tf(bag, kitti, kitti_type, initial_time):
    print("Exporting time dependent transformations")
    if kitti_type.find("raw") != -1:
        for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
            tf_oxts_msg = TFMessage()
            tf_oxts_transform = TransformStamped()
            tf_oxts_transform.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
            tf_oxts_transform.header.frame_id = 'world'
            tf_oxts_transform.child_frame_id = 'base_link'

            transform = (oxts.T_w_imu)
            t = transform[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(transform)
            oxts_tf = Transform()

            oxts_tf.translation.x = t[0]
            oxts_tf.translation.y = t[1]
            oxts_tf.translation.z = t[2]

            oxts_tf.rotation.x = q[0]
            oxts_tf.rotation.y = q[1]
            oxts_tf.rotation.z = q[2]
            oxts_tf.rotation.w = q[3]

            tf_oxts_transform.transform = oxts_tf
            tf_oxts_msg.transforms.append(tf_oxts_transform)

            bag.write('/tf', tf_oxts_msg, tf_oxts_msg.transforms[0].header.stamp)

    elif kitti_type.find("odom") != -1:
        timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        for timestamp, tf_matrix in zip(timestamps, kitti.T_w_cam0):
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = rospy.Time.from_sec(timestamp)
            tf_stamped.header.frame_id = 'world'
            tf_stamped.child_frame_id = 'camera_left'
            
            t = tf_matrix[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(tf_matrix)
            transform = Transform()

            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]

            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped.transform = transform
            tf_msg.transforms.append(tf_stamped)

            bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.from_sec(timestamp)
            odom_msg.header.frame_id = 'world'
            odom_msg.child_frame_id = 'camera_left'
            
            odom_msg.pose.pose.position.x = t[0]
            odom_msg.pose.pose.position.y = t[1]
            odom_msg.pose.pose.position.z = t[2]

            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            bag.write('/odom/camera_left', odom_msg, tf_msg.transforms[0].header.stamp)


def save_velo_data(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir))
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes.append(dt)

    iterable = zip(velo_datetimes, velo_filenames)
    bar = progressbar.ProgressBar()
    for dt, filename in bar(iterable):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, filename)

        # read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))

        # fill pcl msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('i', 12, PointField.FLOAT32, 1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)

        bag.write(topic + '/pointcloud', pcl_msg, t=pcl_msg.header.stamp)

def save_velo_data_odom(bag, kitti, velo_frame_id, topic, initial_time):
    print("Exporting velodyne data")
    timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
    iterable = zip(timestamps, kitti.velo)
    bar = progressbar.ProgressBar()
    for timestamp, scan in bar(iterable):
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(timestamp)
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('i', 12, PointField.FLOAT32, 1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)

        bag.write(topic + '/pointcloud', pcl_msg, t=pcl_msg.header.stamp)


def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = tf.transformations.quaternion_from_matrix(transform)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg


def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_static_transforms(bag, transforms, timestamps):
    print("Exporting static transformations")
    tfm = TFMessage()
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])
        tfm.transforms.append(t)
    for timestamp in timestamps:
        time = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
        bag.write('/tf_static', tfm, t=time)

def save_static_transform_velo(bag, kitti, cam_frame_id,velo_frame_id,initial_time):
    print("Exporting static transformations of Velodyne")
    print("Camera Frame : "+cam_frame_id+", Velodyne frame : "+velo_frame_id)
    print(kitti.T_cam0_velo)
    print("----------------")
    timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
    tfm = TFMessage()
    t = get_static_transform(from_frame_id=cam_frame_id, to_frame_id=velo_frame_id, transform=kitti.T_cam0_velo)
    tfm.transforms.append(t)
    for timestamp in timestamps:
        time = rospy.Time.from_sec(timestamp)
        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
        bag.write('/tf_static', tfm, t=time)

def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)


def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write(topic, twist_msg, t=twist_msg.header.stamp)


def save_data2node(bag, kitti, kitti_type, initial_time):
    CAM2BASE = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                         [- 6.481465826011e-03, 8.051860151134e-03, - 9.999466081774e-01, - 7.337429464231e-02],
                         [9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                         [0, 0, 0, 1]])
    tf_origin = np.array([[0, 0, 1, 0],
                          [-1, 0, 0, 0],
                          [0, -1, 0, 0],
                          [0,  0, 0, 1]])

    if kitti_type.find("odom") != -1:
        timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        assert(len(kitti.T_w_cam0)) == len(kitti.velo)
        assert(len(kitti.velo)) == len(kitti.label)
        print("{} vs {} vs {}. Condition satisfied.".format(len(kitti.T_w_cam0), len(kitti.velo), len(kitti.label)))
        iterable = zip(timestamps, kitti.velo, kitti.label, kitti.T_w_cam0)
        bar = progressbar.ProgressBar()
        count = 0
        for timestamp, scan, label, tf_matrix_cam in bar(iterable):
            tmp = np.matmul(tf_matrix_cam, CAM2BASE)
            tf_matrix = np.matmul(tf_origin, tmp)
            out = node()
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = rospy.Time.from_sec(timestamp)
            tf_stamped.header.frame_id = '/map'
            tf_stamped.child_frame_id = 'camera_left'

            t = tf_matrix[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(tf_matrix)
            transform = Transform()

            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]

            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped.transform = transform
            tf_msg.transforms.append(tf_stamped)

            bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)

            pose_msg = Pose()

            pose_msg.position.x = t[0]
            pose_msg.position.y = t[1]
            pose_msg.position.z = t[2]

            pose_msg.orientation.x = q[0]
            pose_msg.orientation.y = q[1]
            pose_msg.orientation.z = q[2]
            pose_msg.orientation.w = q[3]

            header = Header()
            header.seq = kitti.frame_range[count]
            header.frame_id = "map"
            header.stamp = rospy.Time.from_sec(timestamp)
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      PointField('intensity', 12, PointField.FLOAT32, 1)]

            scan_xyz = scan[:, :3]
            label_float = label.astype(np.float32)
            scan_w_label = np.concatenate((scan_xyz, label_float), axis=1)

            pcl_msg = pcl2.create_cloud(header, fields, scan_w_label)
            pcl_raw_msg = pcl2.create_cloud(header, fields, scan)

            out.header = header
            out.odom = pose_msg
            out.lidar = pcl_msg


            bag.write('/node/combined/optimized', out, out.header.stamp)
            bag.write('/debug/pc_raw', pcl_raw_msg, out.header.stamp)
            bag.write('/debug/pc_label', pcl_msg, out.header.stamp)

            count += 1


def main():
    ABS_PATH = "/media/shapelim/UX960NVMe1/kitti_semantic/dataset"
    SAVE_PATH = "/media/shapelim/UX960NVMe1/kitti_semantic/rosbag_final_ERASOR/seq05"
    parser = argparse.ArgumentParser(description="Convert KITTI dataset to ROS bag file the easy way!")
    # Accepted argument values
    kitti_types = ["raw_synced", "odom_color", "odom_gray", "odom_noimg"]
    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))
    
    parser.add_argument("--kitti_type", default="odom_noimg", choices=kitti_types, help="KITTI dataset type")
    # parser.add_argument("--dir", nargs="?", default = os.getcwd(), help="base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("--dir", default=ABS_PATH, help="base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("--savedir", default=SAVE_PATH, help="base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-t", "--date", default=None, help="date of the raw dataset (i.e. 2011_09_26), option is only for RAW datasets.")
    parser.add_argument("-r", "--drive", default=None, help="drive number of the raw dataset (i.e. 0001), option is only for RAW datasets.")
    parser.add_argument("-s", "--sequence", default="00", choices=odometry_sequences, help="sequence of the odometry dataset (between 00 - 21), option is only for ODOMETRY datasets.")
    parser.add_argument("-i", "--init_stamp", default=0, type=int)
    parser.add_argument("-itv", "--interval", default=2, type=int)
    parser.add_argument("-e", "--end_stamp", default=1000, type=int)
    args = parser.parse_args()

    bridge = CvBridge()
    compression = rosbag.Compression.NONE
    # compression = rosbag.Compression.BZ2
    # compression = rosbag.Compression.LZ4
    
    # CAMERAS
    cameras = [
        (0, 'camera_gray_left', '/kitti/camera_gray_left'),
        (1, 'camera_gray_right', '/kitti/camera_gray_right'),
        (2, 'camera_color_left', '/kitti/camera_color_left'),
        (3, 'camera_color_right', '/kitti/camera_color_right')
    ]


    if args.sequence == None:
        print("Sequence option is not given. It is mandatory for odometry dataset.")
        print("Usage for odometry dataset: kitti2bag {odom_color, odom_gray} [dir] -s <sequence>")
        sys.exit(1)

    # Add [0] is important since the cpp drop the first data.
    init_stamp = args.init_stamp
    final_stamp = args.end_stamp
    interval = args.interval

    frame_range = [init_stamp] + range(init_stamp, final_stamp, interval)
    bag = rosbag.Bag(os.path.join(args.savedir, "{}_{}_to_{}_w_interval_{}_node.bag".format(args.sequence, init_stamp,
                                                                                              final_stamp, interval)),
                                                                                              'w', compression=compression)


    kitti = pykitti.odometry(args.dir, args.sequence, frame_range)
    if not os.path.exists(kitti.sequence_path):
        print('Path {} does not exists. Exiting.'.format(kitti.sequence_path))
        sys.exit(1)

    kitti.load_calib()
    kitti.load_timestamps()

    if len(kitti.timestamps) == 0:
        print('Dataset is empty? Exiting.')
        sys.exit(1)

    if args.sequence in odometry_sequences[:11]:
        print("Odometry dataset sequence {} has ground truth information (poses).".format(args.sequence))
        kitti.load_poses()

    try:
        util = pykitti.utils.read_calib_file(os.path.join(args.dir, 'sequences', args.sequence, 'calib.txt'))
        current_epoch = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
        # Export
        if args.kitti_type.find("gray") != -1:
            used_cameras = cameras[:2]
        elif args.kitti_type.find("color") != -1:
            used_cameras = cameras[-2:]

        kitti.load_velo()
        kitti.load_label()
        velo_frame_id = 'velo_link'
        velo_topic = '/kitti/velo'
        save_data2node(bag, kitti, args.kitti_type, initial_time=current_epoch)
        # save_static_transform_velo(bag, kitti, 'camera_left', velo_frame_id, initial_time=current_epoch)
        # save_velo_data_odom(bag, kitti, velo_frame_id, velo_topic, initial_time=current_epoch)
        #
        # save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=current_epoch)


    finally:
        print("## OVERVIEW ##")
        print(bag)
        bag.close()

if __name__ == '__main__':
    main()
