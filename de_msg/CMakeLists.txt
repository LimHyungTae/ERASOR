cmake_minimum_required(VERSION 2.8.3)
project(de_msg)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    visualization_msgs
    cv_bridge
    message_generation
    sensor_msgs
    geometry_msgs
    )

add_message_files(
    FILES
    node.msg
    mclnode.msg
    satelliteMap.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
   )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES beginner_tutorials
    #  CATKIN_DEPENDS roscpp rospy std_msgs
    #  DEPENDS system_lib
)
