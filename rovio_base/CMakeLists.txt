cmake_minimum_required(VERSION 2.8.3)
project(rovio_base)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)
find_package(CURL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(FILES report.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

## Generate added messages and services with any dependencies listed here
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(Parser src/Parser.cpp)
target_link_libraries(Parser ${catkin_LIBRARIES} ${CURL_LIBRARIES})

add_executable(rovio_base nodes/rovio_base.cpp)
target_link_libraries(rovio_base Parser ${catkin_LIBRARIES})
add_dependencies(rovio_base rovio_base_gencpp)

add_executable(rovio_test nodes/rovio_test.cpp)
target_link_libraries(rovio_test ${catkin_LIBRARIES})
add_dependencies(rovio_test rovio_base_gencpp)
