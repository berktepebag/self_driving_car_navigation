cmake_minimum_required(VERSION 2.8.3)
project(rc_car_2dnav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
  rosbag
  message_generation
  dynamic_reconfigure
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/Commands.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rc_car_2dnav
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(command_controller src/command_controller.cpp)
target_link_libraries(command_controller 
${catkin_LIBRARIES} 
)
add_dependencies(command_controller ${PROJECT_NAME}_gencfg)


add_executable(pi_controller src/pi_controller.cpp)
target_link_libraries(pi_controller 
${catkin_LIBRARIES} 
)

add_executable(trajectory_utilities src/trajectory_utilities.cpp)
target_link_libraries(trajectory_utilities 
${catkin_LIBRARIES} 
)

