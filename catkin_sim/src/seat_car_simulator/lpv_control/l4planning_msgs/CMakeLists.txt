cmake_minimum_required(VERSION 3.5)
project(l4planning_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    CubicSpline.msg
    G2Spline.msg
    G2Path.msg
    Trajectory.msg
    TrajectorySet.msg
    Waypoints.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
