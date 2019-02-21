cmake_minimum_required(VERSION 2.8.3)
project(uav_sim)

# default to release build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_FLAGS "-std=c++14 -mavx2 -mfma -fopenmp")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

add_subdirectory(lib/common_cpp)
add_subdirectory(lib/geometry)

include_directories(include
  lib/common_cpp/include
  lib/geometry/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  )

add_executable(uav_sim
  src/main.cpp
  src/environment.cpp
  src/quadrotor.cpp
  src/bicycle.cpp
  src/sensors.cpp
  src/controller.cpp
  )
target_link_libraries(uav_sim
  common_cpp
  ${YAML_CPP_LIBRARIES}
  ${CERES_LIBRARIES}
  )
