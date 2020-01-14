cmake_minimum_required(VERSION 2.8.3)
project(uav_sim)

# default to release build
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Build type: Release")
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_FLAGS "-std=c++14 -mavx2 -mfma -fopenmp")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)

add_subdirectory(lib/common_cpp)
add_subdirectory(lib/geometry)
add_subdirectory(lib/lin_alg_tools)
add_subdirectory(lib/pb-vi-ekf)

include_directories(include
  lib/common_cpp/include
  lib/geometry/include
  lib/pb-vi-ekf/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  )

add_executable(uav_sim
  src/main.cpp
  src/environment.cpp
  src/quadrotor.cpp
  src/fixed_wing.cpp
  src/fixed_wing_trim.cpp
  src/fixed_wing_base.cpp
  src/sensors.cpp
  src/quad_control.cpp
  src/fixed_wing_control.cpp
  src/fw_lqr.cpp
  src/fw_ekf.cpp
  src/gimbal.cpp
  src/gmbl_ctrl_pid.cpp
  src/gmbl_ekf.cpp
  src/bicycle.cpp
  src/bicycle_ctrl_pid.cpp
  )
target_link_libraries(uav_sim
  common_cpp
  geometry
  lin_alg_tools
  pb_vi_ekf
  ${YAML_CPP_LIBRARIES}
  ${CERES_LIBRARIES}
  )

add_executable(fw_ekf_test
  src/fw_ekf_test.cpp
  src/fw_ekf.cpp
  )
target_link_libraries(fw_ekf_test
  common_cpp
  geometry
  ${YAML_CPP_LIBRARIES}
  )

add_executable(test_gmbl_ekf 
  test/gmbl_ekf_test.cpp
  src/gmbl_ekf.cpp
)
target_link_libraries(test_gmbl_ekf
  gtest
  gtest_main
  pthread
  ${GTEST_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)