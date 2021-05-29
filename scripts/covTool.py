#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import sys

repubO = rospy.Publisher('/odom/covadd', Odometry, queue_size=10)
repubI = rospy.Publisher('/imu/covadd', Imu, queue_size=10)
odomCov     = np.array([0.0]*9)
imuOriCov   = np.array([0.0]*9)
imuGyrCov   = np.array([0.0]*9)
imuAccCov   = np.array([0.0]*9)
yaw_before  = 0

def odometryCb(msg):
    global yaw_before
    msg_wcov = msg
    p_cov = np.array([0.0]*36).reshape(6,6)
    p_cov[0,0] = odomCov[0]
    p_cov[0,1] = odomCov[1]
    p_cov[0,5] = odomCov[2]
    p_cov[1,0] = odomCov[3]
    p_cov[1,1] = odomCov[4]
    p_cov[1,5] = odomCov[5]
    p_cov[5,0] = odomCov[6]
    p_cov[5,1] = odomCov[7]
    p_cov[5,5] = odomCov[8]
    msg_wcov.pose.pose.position.x = msg_wcov.pose.pose.position.x * 0.9384
    msg_wcov.pose.pose.position.y = msg_wcov.pose.pose.position.y * 0.9384
    msg_wcov.pose.pose.position.z = msg_wcov.pose.pose.position.z * 0.9384

    quaternion = (
        msg_wcov.pose.pose.orientation.x,
        msg_wcov.pose.pose.orientation.y,
        msg_wcov.pose.pose.orientation.z,
        msg_wcov.pose.pose.orientation.w)
    euler =  euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

#    yaw = yaw_before + (yaw-yaw_before)*0.92
    quaternion = quaternion_from_euler(roll, pitch, yaw)
#    yaw_before = yaw
    #type(pose) = geometry_msgs.msg.Pose
    msg_wcov.pose.pose.orientation.x = quaternion[0]
    msg_wcov.pose.pose.orientation.y = quaternion[1]
    msg_wcov.pose.pose.orientation.z = quaternion[2]
    msg_wcov.pose.pose.orientation.w = quaternion[3]

    msg_wcov.pose.covariance = tuple(p_cov.ravel().tolist())
    repubO.publish(msg_wcov)

def imuCb(msg):
    msg_wcov = msg
    msg_wcov.orientation_covariance         = imuOriCov
    msg_wcov.angular_velocity_covariance    = imuGyrCov
    msg_wcov.linear_acceleration_covariance = imuAccCov
    repubI.publish(msg_wcov)

if __name__ == '__main__':
    rospy.init_node('odometryCovar', anonymous=False) #make node
    rospy.Subscriber('/odom',Odometry,odometryCb)
    rospy.Subscriber('/imu',Imu,imuCb)
    odomCov = rospy.get_param("/covariance/odom",[0,0,0,0,0,0,0,0,0])
    imuOriCov = rospy.get_param("/covariance/imu_ori",[0,0,0,0,0,0,0,0,0])
    imuGyrCov = rospy.get_param("/covariance/imu_gyr",[0,0,0,0,0,0,0,0,0])
    imuAccCov = rospy.get_param("/covariance/imu_acc",[0,0,0,0,0,0,0,0,0])
    rospy.spin()
