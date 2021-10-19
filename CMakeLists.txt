 cmake_minimum_required(VERSION 2.8.3)
project(erasor)
set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    nav_msgs
    roslaunch
    cv_bridge
    pcl_conversions
    pcl_ros
    geometry_msgs
    laser_geometry
    sensor_msgs
)

add_message_files(
   FILES
   node.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)


find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)


catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  include
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

#add_executable(marker src/pub_marker.cpp ${unavlib_source})
#target_link_libraries(marker ${catkin_LIBRARIES})
#add_dependencies(marker erasor_generate_messages_cpp)

###############
 # Deprecated!
#add_executable(fill_intensity src/test_codes/fill_removert_intensity.cpp ${unavlib_source})
#target_link_libraries(fill_intensity ${catkin_LIBRARIES})
#add_dependencies(fill_intensity erasor_generate_messages_cpp)
#
#
#add_executable(compare_map src/test_codes/compare_map.cpp ${unavlib_source})
#target_link_libraries(compare_map ${catkin_LIBRARIES})
#add_dependencies(compare_map erasor_generate_messages_cpp)
#
#add_executable(compare_complement src/test_codes/compare_complement.cpp ${unavlib_source})
#target_link_libraries(compare_complement ${catkin_LIBRARIES})
#add_dependencies(compare_complement erasor_generate_messages_cpp)
#
#
#add_executable(mapviz src/test_codes/viz_map.cpp ${unavlib_source})
#target_link_libraries(mapviz ${catkin_LIBRARIES})
#add_dependencies(mapviz erasor_generate_messages_cpp)
#
#
#add_executable(merge_map src/test_codes/merge_map.cpp ${unavlib_source})
#target_link_libraries(merge_map ${catkin_LIBRARIES})
#add_dependencies(merge_map erasor_generate_messages_cpp)
################

# Online Map refinement
set(erasor_src
        src/online_map_updater/src/OnlineMapUpdater.cpp
        src/online_map_updater/src/erasor.cpp
        src/online_map_updater/src/erasor_utils.cpp
)

add_executable(offline_map_updater src/online_map_updater/erasor_main.cpp ${erasor_src})
target_link_libraries(offline_map_updater ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(offline_map_updater erasor_generate_messages_cpp)

add_executable(main_in_your_env src/online_map_updater/main_in_your_env.cpp ${erasor_src})
target_link_libraries(main_in_your_env ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(main_in_your_env erasor_generate_messages_cpp)

 add_executable(compare_map src/compare_map.cpp src/online_map_updater/src/erasor_utils.cpp)
target_link_libraries(compare_map ${catkin_LIBRARIES})

add_executable(kitti_mapgen src/mapgen/main.cpp src/mapgen/mapgen.hpp src/online_map_updater/src/erasor_utils.cpp)
target_link_libraries(kitti_mapgen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(kitti_mapgen erasor_generate_messages_cpp)
