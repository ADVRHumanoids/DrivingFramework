cmake_minimum_required(VERSION 2.8.3)
project(hierarchical_control)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  simple_log
  eigen_utils
  point_handling
  collision_model
  robot_class
  basic_controllers
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
	include
	${RBDL_INCLUDE_DIR}
  LIBRARIES 
	hierarchical_controller 
	controller_tasks
	${RBDL_LIBRARY}
  CATKIN_DEPENDS 
	roscpp 
	eigen_utils 
	collision_model 
	point_handling
        basic_controllers
        robot_class
  DEPENDS 
	Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
   add_library(controller_tasks
     src/controller_task.cpp
     src/self_collision_task.cpp
     src/cartesian_world_task.cpp
     src/orientation_world_task.cpp
     src/self_collisions_task.cpp
     src/joint_positions_task.cpp
     src/center_of_mass_task.cpp
     src/constraints_task.cpp
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
   add_dependencies(controller_tasks ${catkin_EXPORTED_TARGETS})

   target_link_libraries(controller_tasks
     ${catkin_LIBRARIES} 
     ${RBDL_LIBRARY}
   )

   add_library(hierarchical_controller
     src/hierarchical_controller.cpp
#     src/hierarchical_controller_continous.cpp
   )
##   Add cmake target dependencies of the library
##   as an example, code may need to be generated before libraries
##   either from message generation or dynamic reconfigure

   add_dependencies(hierarchical_controller ${catkin_EXPORTED_TARGETS})

   target_link_libraries(hierarchical_controller
     ${catkin_LIBRARIES} 
     ${RBDL_LIBRARY}
   )


#############
## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS controller_tasks hierarchical_controller
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
 find_package(tests_common REQUIRED)
 find_package(centauro REQUIRED)

 LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
 find_package(RBDL REQUIRED)
 find_package(Eigen3 REQUIRED)
 include_directories(${tests_common_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS})

 catkin_add_gtest(controller_task-test test/src/tasks/test_controller_task.cpp)

 if(TARGET controller_task-test)
  add_dependencies(controller_task-test ${catkin_EXPORTED_TARGETS} ${tests_common_EXPORTED_TARGETS})
  target_link_libraries(controller_task-test
        controller_tasks
        ${catkin_LIBRARIES}
        ${tests_common_LIBRARIES}
   )
 endif()

 catkin_add_gtest(cartesian_world_task-test test/src/tasks/test_cartesian_world_task.cpp)

 if(TARGET cartesian_world_task-test)
  add_dependencies(cartesian_world_task-test ${catkin_EXPORTED_TARGETS} ${tests_common_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(cartesian_world_task-test
        controller_tasks
        ${RBDL_LIBRARY}
        ${catkin_LIBRARIES}
        ${tests_common_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

 catkin_add_gtest(self_collision_task-test test/src/tasks/test_self_collision_task.cpp)

 if(TARGET self_collision_task-test)
  add_dependencies(self_collision_task-test ${catkin_EXPORTED_TARGETS} ${tests_common_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(self_collision_task-test
        controller_tasks
        ${RBDL_LIBRARY}
        ${catkin_LIBRARIES}
        ${tests_common_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

 catkin_add_gtest(self_collisions_task-test test/src/tasks/test_self_collisions_task.cpp)

 if(TARGET self_collisions_task-test)
  add_dependencies(self_collisions_task-test ${catkin_EXPORTED_TARGETS} ${tests_common_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(self_collisions_task-test
        controller_tasks
        ${RBDL_LIBRARY}
        ${catkin_LIBRARIES}
        ${tests_common_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

 catkin_add_gtest(hierarchical_control-test test/src/test_hierarchical_control.cpp)

 if(TARGET hierarchical_control-test)
  add_dependencies(hierarchical_control-test ${catkin_EXPORTED_TARGETS} ${tests_common_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(hierarchical_control-test
        hierarchical_controller
        controller_tasks
        ${RBDL_LIBRARY}
        ${catkin_LIBRARIES}
        ${tests_common_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

 catkin_add_gtest(center_of_mass_task-test test/src/tasks/test_center_of_mass_task.cpp)

 if(TARGET center_of_mass_task-test)
  add_dependencies(center_of_mass_task-test ${catkin_EXPORTED_TARGETS} ${tests_common_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(center_of_mass_task-test
        hierarchical_controller
        controller_tasks
        ${RBDL_LIBRARY}
        ${catkin_LIBRARIES}
        ${tests_common_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

endif(CATKIN_ENABLE_TESTING)

