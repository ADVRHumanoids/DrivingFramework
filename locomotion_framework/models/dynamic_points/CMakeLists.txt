cmake_minimum_required(VERSION 2.8.3)
project(mwoibn_dynamic_points)


# The version number, project name, add c++ flags
set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

add_definitions(-std=c++11)

option(LOGGER OFF)


find_package(catkin REQUIRED robot_class dynamic_models point_handling)


catkin_package(
  INCLUDE_DIRS
			include
			${catkin_INCLUDE_DIRS}
            ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include
  LIBRARIES
      mwoibn_dynamic_points
			${catkin_LIBRARIES}
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Declare a C++ library
add_library(mwoibn_dynamic_points
                src/force.cpp
                src/linear_force.cpp
                src/torus.cpp
                src/torus_velocity.cpp
)

add_dependencies(mwoibn_dynamic_points ${catkin_EXPORTED_TARGETS})

target_link_libraries(mwoibn_dynamic_points
	${catkin_LIBRARIES}
)

## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS mwoibn_dynamic_points
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/dynamic_points
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_class.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

if(CATKIN_ENABLE_TESTING)
 find_package(roslib REQUIRED)
 find_package(centauro REQUIRED)

 include_directories(test/include)

 #catkin_add_gtest(${PROJECT_NAME}-test test/src/test_robot_class.cpp)

 if(TARGET ${PROJECT_NAME}-test)
  include_directories(${roslib_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS})
  add_dependencies(${PROJECT_NAME}-test ${catkin_EXPORTED_TARGETS} ${roslib_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}-test
        ${PROJECT_NAME}
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

 #catkin_add_gtest(contacts-test test/src/test_contacts.cpp)

 if(TARGET contacts-test)
  include_directories(${roslib_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS})
  add_dependencies(contacts-test ${catkin_EXPORTED_TARGETS} ${roslib_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(contacts-test
        ${PROJECT_NAME}
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

 #catkin_add_gtest(actuators-test test/src/test_actuators.cpp)

 if(TARGET actuators-test)
  include_directories(${roslib_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS})
  add_dependencies(actuators-test ${catkin_EXPORTED_TARGETS} ${roslib_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(actuators-test
        ${PROJECT_NAME}
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

 catkin_add_gtest(center_of_mass-test test/src/test_center_of_mass.cpp)

 if(TARGET center_of_mass-test)
  include_directories(${roslib_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS})
  add_dependencies(center_of_mass-test ${catkin_EXPORTED_TARGETS} ${roslib_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS})
  target_link_libraries(center_of_mass-test
        ${PROJECT_NAME}
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${centauro_LIBRARIES}
   )
 endif()

endif(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(tests_${PROJECT_NAME} tests/mynode.test src/test/test_mynode.cpp)
#  target_link_libraries(tests_mynode ${catkin_LIBRARIES})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
