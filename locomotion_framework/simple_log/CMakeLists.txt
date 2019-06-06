cmake_minimum_required(VERSION 2.8.3)
project(simple_log)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)# COMPONENTS

catkin_package(
  INCLUDE_DIRS 
        include 
)

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
