cmake_minimum_required(VERSION 3.0.2)
project(locate)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  serial
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs sensor_msgs serial
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(locate
  src/locate.cpp
  src/odom.cpp
  src/imu.cpp
  src/turret.cpp
)

add_dependencies(locate ${catkin_EXPORTED_TARGETS})

target_link_libraries(locate
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
