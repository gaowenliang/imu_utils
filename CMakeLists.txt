cmake_minimum_required(VERSION 2.8.3)
project(imu_utils)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    code_utils
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(gyro_test
    src/gyro_an.cpp
    src/gyro_lib/allan.cpp
    src/gyro_lib/allan_acc.cpp
    src/gyro_lib/fitallan.cpp
    )

target_link_libraries(gyro_test dw ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(gyro_test2
    src/gyro_an2.cpp
    src/gyro_lib/allan.cpp
    src/gyro_lib/allan_acc.cpp
    src/gyro_lib/fitallan.cpp
    )

target_link_libraries(gyro_test2 dw ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
