cmake_minimum_required(VERSION 2.8.3)
project(hierarchical_control)

add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  simple_log
  eigen_utils
  point_handling
  robot_point_handling
#  collision_model
  robot_class
  basic_controllers
  mwoibn_dynamic_points
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )

catkin_package(
  INCLUDE_DIRS
	    include
  LIBRARIES
	    hierarchical_controller
      controller_tasks
      controller_actions
  CATKIN_DEPENDS
    	roscpp
    	eigen_utils
      mwoibn_std_utils
    #	collision_model
    	point_handling
    	robot_point_handling
      basic_controllers
      robot_class
      mwoibn_dynamic_points
  DEPENDS
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
)

## Declare a C++ library
   add_library(controller_tasks
     src/tasks/angle.cpp
     src/tasks/aggravated.cpp
     src/tasks/controller_task.cpp
#     src/tasks/self_collision_task.cpp
     src/tasks/cartesian_world_task.cpp
     src/tasks/orientation_world_task.cpp
#     src/tasks/self_collisions_task.cpp
     src/tasks/joint_positions_task.cpp
     src/tasks/center_of_mass_task.cpp
     src/tasks/center_of_mass_task_v2.cpp
     src/tasks/constraints_task.cpp
)

  add_dependencies(controller_tasks ${catkin_EXPORTED_TARGETS})
  target_link_libraries(controller_tasks
     ${catkin_LIBRARIES}
  )



  add_library(controller_actions
    src/actions/merge.cpp
    src/actions/angles_merge.cpp

    src/actions/merge/end.cpp
    src/actions/merge/front.cpp
    src/actions/merge/local.cpp
    src/actions/merge/replace.cpp
)
 add_dependencies(controller_actions ${catkin_EXPORTED_TARGETS})
 target_link_libraries(controller_actions
    ${catkin_LIBRARIES}
 )



  add_library(actions_merge
     src/actions/merge/front.cpp
     src/actions/merge/end.cpp
     src/actions/merge/local.cpp
#     src/actions/merge/replace_map.cpp
     src/actions/merge/replace.cpp
 )
  add_dependencies(actions_merge ${catkin_EXPORTED_TARGETS})
  target_link_libraries(actions_merge
     ${catkin_LIBRARIES}
  )


  add_library(hierarchical_controller
     src/controllers/default.cpp
     src/controllers/continous.cpp
     src/controllers/wheels.cpp
     src/controllers/actions.cpp
   )
   add_dependencies(hierarchical_controller ${catkin_EXPORTED_TARGETS})

   target_link_libraries(hierarchical_controller
     ${catkin_LIBRARIES}
   )


#############
## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS controller_tasks hierarchical_controller controller_actions
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
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
        ${catkin_LIBRARIES}
        ${tests_common_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

endif(CATKIN_ENABLE_TESTING)
