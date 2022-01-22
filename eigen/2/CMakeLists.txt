project(matrix)
cmake_minimum_required(VERSION 3.10)
set(SRC_LIST matrix.cpp)
find_package(Eigen3)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
add_executable(matrix ${SRC_LIST})

