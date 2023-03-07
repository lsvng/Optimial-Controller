cmake_minimum_required(VERSION 3.2)
project(OPTIMAL_CONTROLLER LANGUAGES CXX C)

# Set C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Build.
include_directories(
  include
  utilities
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_lib SHARED
  source/StateSpaceModel.cpp
  source/LQR.cpp
  source/MPC.cpp
  source/PID.cpp
)

target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen)

# Excecutable
add_executable(${PROJECT_NAME}_LQR
  example/LQR_Example.cpp
)

target_link_libraries(${PROJECT_NAME}_LQR ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_MPC
  example/MPC_Example.cpp
)

target_link_libraries(${PROJECT_NAME}_MPC ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_PID
  example/PID_Example.cpp
)

target_link_libraries(${PROJECT_NAME}_PID ${PROJECT_NAME}_lib)

# Unit Test
add_executable(${PROJECT_NAME}_PID_Test
  test/PID_Test.cpp
)
target_link_libraries(${PROJECT_NAME}_PID_Test
  ${GTEST_LIBRARIES}
  ${PROJECT_NAME}_lib
  pthread
)

# Install.
install(TARGETS ${PROJECT_NAME}_LQR ${PROJECT_NAME}_MPC ${PROJECT_NAME}_PID
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

INSTALL (DIRECTORY include/
  DESTINATION /usr/local/include
  FILES_MATCHING PATTERN "*.hpp*"
)
