cmake_minimum_required(VERSION 3.0.2)
project(drone_path_planner)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

 add_message_files(
 	FILES
   teleopData.msg
 )

 generate_messages(
  DEPENDENCIES
   std_msgs
 )


find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

Catkin_package(
  CATKIN_DEPENDS message_runtime 
  )