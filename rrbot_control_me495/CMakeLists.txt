cmake_minimum_required(VERSION 2.8.3)
project(rrbot_control_me495)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  controller_manager
  transmission_interface
  hardware_interface
  )

add_message_files(DIRECTORY msg
  FILES
  RRBotConfig.msg
  )

generate_messages(
  DEPENDENCIES
)

# Boost:
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

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
  CATKIN_DEPENDS gazebo_msgs roscpp message_runtime sensor_msgs std_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  ${GAZEBO_CXX_FLAGS} -std=c++11")

add_library(rrbot_custom src/rrbot_custom_plugin.cpp)
target_link_libraries(rrbot_custom ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rrbot_custom ${PROJECT_NAME}_generate_messages_cpp)
