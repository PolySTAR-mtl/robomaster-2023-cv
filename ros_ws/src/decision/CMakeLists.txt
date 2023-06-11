cmake_minimum_required(VERSION 3.0.2)
project(decision)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  detection
  serial
  tf2
  tf2_ros
)


find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS detection serial tf2 tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(simple_tracker
  src/simple_tracker.cpp
)

target_link_libraries(simple_tracker
  ${catkin_LIBRARIES}
)

add_dependencies(simple_tracker ${catkin_EXPORTED_TARGETS})

# Weighted tracker

add_executable(weighted_tracker
  src/weighted_tracker.cpp
)

target_link_libraries(weighted_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(weighted_tracker ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
