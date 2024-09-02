cmake_minimum_required(VERSION 3.10)

project(ekf_pose)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(Pangolin_DIR ${CMAKE_SOURCE_DIR}/third_party/Pangolin/build)
find_package(Pangolin REQUIRED)

set(GeographicLib_DIR ${CMAKE_SOURCE_DIR}/third_party/geographiclib/build)
find_package(GeographicLib REQUIRED)
message(STATUS "geo_VERSION: ${GeographicLib_VERSION}")
message(STATUS "geo_INCLUDE_DIRS: ${GeographicLib_INCLUDE_DIRS}")
message(STATUS "geo_LIBRARIES: ${GeographicLib_LIBRARIES}")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

include_directories(
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
    include
)

add_executable(ros_ekf_pose
    src/main.cpp
    src/ros_node.cpp
    src/ekf.cpp
    src/viewer.cpp
)
target_link_libraries(ros_ekf_pose
  ${catkin_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)