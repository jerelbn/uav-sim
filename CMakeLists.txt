cmake_minimum_required (VERSION 2.8.11)
project (uav_sim)

# default to release build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_FLAGS "-std=c++11 -openmp")

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include
  lib/common_cpp/include
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(common_cpp lib/common_cpp/src/common.cpp)

add_executable(uav_sim
  src/main.cpp
)
target_link_libraries(uav_sim common_cpp ${YAML_CPP_LIBRARIES} stdc++fs)
