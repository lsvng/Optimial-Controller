cmake_minimum_required(VERSION 3.2)
project(CONTROLLER LANGUAGES CXX C)

# Set C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Build.
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Excecutable
add_executable(${PROJECT_NAME}_LQR
  source/LQR.cpp
  example/LQR_Example.cpp
)

target_link_libraries(${PROJECT_NAME}_LQR Eigen3::Eigen)

add_executable(${PROJECT_NAME}_MPC
  source/MPC.cpp
  example/MPC_Example.cpp
)

target_link_libraries(${PROJECT_NAME}_MPC Eigen3::Eigen)

add_executable(${PROJECT_NAME}_PID
  source/PID.cpp
  example/PID_Example.cpp
)

target_link_libraries(${PROJECT_NAME}_PID Eigen3::Eigen)

# Unit Test
add_executable(${PROJECT_NAME}_PID_Test
  source/PID.cpp
  test/PID_Test.cpp
)
target_link_libraries(${PROJECT_NAME}_PID_Test
  ${GTEST_LIBRARIES}
  Eigen3::Eigen
  pthread
)
