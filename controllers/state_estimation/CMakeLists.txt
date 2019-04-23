cmake_minimum_required(VERSION 2.8.3)
project(state_estimation)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND CATKIN_DEPENDS_SET robot_class robot_point_handling filters modules)
LIST( APPEND CATKIN_DEPENDS_SET dynamic_models mwoibn_dynamic_points)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS 
        include 
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../locomotion_framework/third_party/range-v3/include
  LIBRARIES
        mgnss_state_estimation
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../locomotion_framework/third_party/range-v3/include
)

add_library(mgnss_state_estimation
   #src/odometry.cpp
   src/odometry_v2.cpp
   src/odometry_v3.cpp

   src/ground_forces.cpp
   src/acceleration_test.cpp
)

add_dependencies(mgnss_state_estimation
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mgnss_state_estimation
   ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
 install(TARGETS mgnss_state_estimation
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/mgnss/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
