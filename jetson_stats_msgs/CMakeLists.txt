cmake_minimum_required(VERSION 2.4.6)

project(jetson_stats_msgs)

find_package(catkin REQUIRED 
  COMPONENTS 
    genmsg
    message_generation
    std_msgs
)

add_message_files(
  FILES
    JetsonStats.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)
