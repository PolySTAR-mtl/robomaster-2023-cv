cmake_minimum_required(VERSION 3.0.2)
project(serial)

# Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## ----- Messages ----- ##

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HP.msg
  Rune.msg
  SwitchOrder.msg
  Target.msg
  Movement.msg
  GameStatus.msg
  GameStage.msg
  TurretFeedback.msg
  PositionFeedback.msg
  Shoot.msg
)

generate_messages()


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
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

add_library(serial_spinner src/serial_spinner)

add_executable(serial_interface
  src/serial_interface.cpp
)

target_link_libraries(serial_interface
  ${catkin_LIBRARIES}
  serial_spinner
)

add_dependencies(serial_interface ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()


catkin_add_gtest(serialize test/serialize.cpp)
target_link_libraries(serialize serial_spinner ${catkin_LIBRARIES})

catkin_add_gtest(deserialize test/deserialize.cpp)
target_link_libraries(deserialize serial_spinner ${catkin_LIBRARIES})
