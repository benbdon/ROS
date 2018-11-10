cmake_minimum_required(VERSION 2.8.3)
project(me495_hw1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  )

add_message_files(
  FILES
  ME495Pub.msg
)

add_service_files(
  FILES
  ME495Srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES me495_hw1
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_cl_demo src/ros_cl_demo.cpp)
add_dependencies(ros_cl_demo me495_hw1_generate_messages_cpp)
target_link_libraries(ros_cl_demo ${catkin_LIBRARIES})
