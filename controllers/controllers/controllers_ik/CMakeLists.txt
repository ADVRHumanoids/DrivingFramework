cmake_minimum_required(VERSION 2.8.3)
project(mgnss_controllers_ik)


set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

#LIST( APPEND INSTALL_TARGETS mgnss_controllers_wheels mgnss_controllers_centralized mgnss_controllers_joint_space)
LIST( APPEND INSTALL_TARGETS mgnss_ik )
LIST( APPEND CATKIN_DEPENDS_SET hierarchical_control dynamic_models modules mgnss_higher_level robot_class)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
          ${PROJECT_SOURCE_DIR}/../../../locomotion_framework/third_party/range-v3/include
  LIBRARIES
        mgnss_ik
  CATKIN_DEPENDS
	${CATKIN_DEPENDS_SET}
  DEPENDS
	${DEPENDS_OPTIONAL}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../../locomotion_framework/third_party/range-v3/include
)

add_library(mgnss_ik

   src/ik_base.cpp
   src/upper_body_IK.cpp

 )

add_dependencies(mgnss_ik ${catkin_EXPORTED_TARGETS})

target_link_libraries(mgnss_ik
   ${catkin_LIBRARIES}
 )

#MESSAGE( WARNING "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX} )
#MESSAGE( WARNING "CMAKE_DEVEL_PREFIX: " ${CMAKE_DEVEL_PREFIX} )

## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/mgnss/controllers
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
