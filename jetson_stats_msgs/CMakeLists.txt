cmake_minimum_required(VERSION 2.4.6)

project(jetson_stats_msgs)

find_package(catkin REQUIRED 
  COMPONENTS 
    rospy
    genmsg
    std_msgs
    message_generation
)

add_message_files(
  FILES
    JetsonStats.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
)
