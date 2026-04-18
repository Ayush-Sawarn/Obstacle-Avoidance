cmake_minimum_required(VERSION 3.0.2)
project(move_object)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(model_push src/model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})
