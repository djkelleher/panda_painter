cmake_minimum_required(VERSION 2.8.3)
project(panda_server)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf
  tf_conversions
  eigen_conversions
  kdl_conversions
  panda_moveit_config
  franka_description
  franka_control
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_msgs
  trajectory_msgs
  message_generation
  message_runtime
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  message_filters
)

add_message_files(
  FILES
  Brush.msg
  Canvas.msg
  Waypoint.msg
  CartTrajCmd.msg
)
add_service_files(
   FILES
   CartPose.srv
   CartTraj.srv
   Home.srv
   GripperPos.srv
   Grasp.srv
   JointPos.srv
   SetCanvas.srv
   SetWorkspace.srv
   ScaleLimits.srv
   ChangeCollisionBehavior.srv
   KDLCartTraj.srv
)
generate_messages(
   DEPENDENCIES
   std_msgs
   moveit_msgs
   geometry_msgs
   moveit_msgs
   trajectory_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    rospy
    panda_moveit_config
    franka_control
    franka_description
    std_msgs
    geometry_msgs
    sensor_msgs
    moveit_msgs
    trajectory_msgs
    message_generation
    message_runtime
    moveit_core
    moveit_ros_planning_interface
    message_filters
)