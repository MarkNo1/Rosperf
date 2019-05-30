cmake_minimum_required(VERSION 3.0)
project(rosperf)

# Include ros compliancy
include(ros_compliancy.cmake)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nodelet
        message_generation
        message_runtime
        rosnodelets
        )

add_message_files(FILES  Byte.msg Frame.msg Signal.msg)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES  std_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros-performance-test
  CATKIN_DEPENDS std_msgs message_runtime
 # DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

##############
##   Nodes  ##
##############

# SENDER
add_executable(sender app/sender.cc src/Sender.cc)
add_dependencies(sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sender ${catkin_LIBRARIES} )
# RECEIVER
add_executable(receiver app/receiver.cc src/Receiver.cc)
add_dependencies(receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(receiver ${catkin_LIBRARIES} )

##############
## Nodelets ##
##############
    ############################  Sender  ############################
    add_library(Sender_nodelet src/nodelets/Sender.cc)
    add_dependencies(Sender_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(Sender_nodelet ${catkin_LIBRARIES})
    #|End Nodelets|