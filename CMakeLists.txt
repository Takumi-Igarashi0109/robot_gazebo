cmake_minimum_required(VERSION 2.8.3)
project(robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  rospy
  tf
  pcl_conversions
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wj_gazebo
CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf_converter src/odom_tf_converter.cpp)
add_executable(pointcloud_tf_changer_node src/pointcloud_tf_changer.cpp)

target_link_libraries(odom_tf_converter
  ${catkin_LIBRARIES}
)
target_link_libraries(pointcloud_tf_changer_node
  ${catkin_LIBRARIES}
)

# add_executable(sim1_odom_tf_converter src/sim1_odom_tf_converter.cpp)

# target_link_libraries(sim1_odom_tf_converter
#   ${catkin_LIBRARIES}
# )