cmake_minimum_required(VERSION 3.5.1)
project(mgnss_higher_level)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )


set(INSTALL_TARGETS "")
set(CATKIN_DEPENDS_SET "")
set(CATKIN_DEPENDS_BUILD "")
set(DEPENDS_OPTIONAL "")

LIST( APPEND INSTALL_TARGETS steering support_shaping)
LIST( APPEND CATKIN_DEPENDS_SET hierarchical_control robot_class eiquadprog)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS_SET}
  ${CATKIN_DEPENDS_BUILD}
)


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../locomotion_framework/third_party/range-v3/include
  LIBRARIES
        steering
        support_shaping
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


add_library(steering
   src/steering_v2.cpp
   src/steering_v4.cpp
   src/steering_v5.cpp
   src/steering_v6.cpp
   src/steering_v7.cpp
   src/steering_v8.cpp
   src/steering_reactif.cpp
   src/steering_shape.cpp
#   src/steering_reactif_zmp.cpp
 )
 add_dependencies(steering ${catkin_EXPORTED_TARGETS})

 target_link_libraries(steering
    ${catkin_LIBRARIES}
  )


 add_library(support_shaping
    src/qp/constraints/joint_constraint_v2.cpp
    src/qp/constraints/merge.cpp

    src/qp/tasks/support_shaping_v4_0.cpp
    src/qp/tasks/support_shaping_v5.cpp
    #src/qp/tasks/qr_tracking.cpp
    src/qp/tasks/qr_task.cpp
    src/qp/tasks/qr_joint_space_v2.cpp
    src/qp/tasks/qr_joint_space_v3.cpp
    src/qp/tasks/qp_aggravated.cpp
    src/state_machine.cpp
    src/state_machine_II.cpp

  )
  add_dependencies(support_shaping ${catkin_EXPORTED_TARGETS})

  target_link_libraries(support_shaping
     ${catkin_LIBRARIES}
   )
target_compile_features(support_shaping PRIVATE cxx_range_for)


 ## Mark executables and/or libraries for installation
  install(TARGETS ${INSTALL_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

 # Mark cpp header files for installation
  install(DIRECTORY include/mgnss/higher_level
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mgnss/
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
