cmake_minimum_required(VERSION 2.8.3)
project(mwoibn_std_utils)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${PROJECT_NAME}
)

install(DIRECTORY include/mwoibn/std_utils
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
