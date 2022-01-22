cmake_minimum_required(VERSION 3.10)
project(eigen_test)
set(SRC_LIST main.cpp)
find_package(Eigen3)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
add_executable(eigen_test ${SRC_LIST})
