cmake_minimum_required(VERSION 2.8.3)
project(mgnss_utils)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

option(VISUALZATION_TOOLS ON)

if(VISUALIZATION_TOOLS)

 add_definitions(-DVISUALIZATION_TOOLS)
 LIST( APPEND INSTALL_TARGETS real_robot_tracker)
 LIST( APPEND CATKIN_DEPENDS_SET visualization_tools robot_point_handling roscpp robot_class_ros_extension )

endif(VISUALIZATION_TOOLS)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS 
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(VISUALIZATION_TOOLS)

add_executable(real_robot_tracker src/real_robot_tracker.cpp)
add_dependencies(real_robot_tracker ${catkin_EXPORTED_TARGETS})
target_link_libraries(real_robot_tracker
    ${catkin_LIBRARIES}
 )

endif(VISUALIZATION_TOOLS)

## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
# install(DIRECTORY include/mgnss/utils/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
