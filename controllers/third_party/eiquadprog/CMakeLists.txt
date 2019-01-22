cmake_minimum_required(VERSION 2.8.3)
project(eiquadprog)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )


find_package(catkin REQUIRED COMPONENTS
)


catkin_package(
  INCLUDE_DIRS include 
)


# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/
   #FILES_MATCHING PATTERN "*.hh"
   PATTERN ".svn" EXCLUDE
 )
