cmake_minimum_required(VERSION 2.8.3)
project(uav_sim)

# default to release build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_FLAGS "-std=c++14 -mavx -mfma")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include
  lib/common_cpp/include
  ${Eigen3_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(common_cpp lib/common_cpp/src/common.cpp)

add_executable(uav_sim
  src/main.cpp
  src/environment.cpp
  src/quadrotor.cpp
  src/sensors.cpp
  src/controller.cpp
  src/ekf/ekf.cpp
)
target_link_libraries(uav_sim common_cpp ${YAML_CPP_LIBRARIES} stdc++fs)

add_executable(jacobians_test
  src/ekf/jacobians_test.cpp
  src/ekf/ekf.cpp
)
target_link_libraries(jacobians_test common_cpp ${YAML_CPP_LIBRARIES} stdc++fs)
