cmake_minimum_required(VERSION 2.8.3)
project(robot_class)


# The version number, project name, add c++ flags
set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)
set (LOG_INFO ${PROJECT_NAME})

add_definitions(-std=c++11)

option(LOGGER OFF)

message(STATUS " Flags:")
message(STATUS " LOGGER ${LOGGER}")

set(LOGGER_INCLUDE_DIRS "")
set(LOGGER_LIBRARIES "")
set(LOGGER_EXPORTED_TARGETS "")

find_package(catkin REQUIRED point_handling urdf srdfdom common eigen_utils) 

if(LOGGER)
find_package(simple_log REQUIRED)
LIST(APPEND LOGGER_LIBRARIES 
	${simple_log_LIBRARIES}
)
LIST(APPEND LOGGER_INCLUDE_DIRS 
	${simple_log_INCLUDE_DIRS}
)
LIST(APPEND LOGGER_EXPORTED_TARGETS 
	${simple_log_EXPORTED_TARGETS}
)
endif(LOGGER)


LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
find_package(RBDL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(YamlCpp REQUIRED)

catkin_package(
  INCLUDE_DIRS 
	include 
	${catkin_INCLUDE_DIRS} 
	${RBDL_INCLUDE_DIR} 
	${RBDL_URDFReader_INCLUDE_DIR} 
	${LOGGER_INCLUDE_DIRS}
  LIBRARIES 
	robot_class
	${catkin_LIBRARIES} 
	${RBDL_LIBRARY} 
	${RBDL_URDFReader_LIBRARY} 
	${LOGGER_LIBRARIES}
  DEPENDS 
	Eigen3
        YamlCpp
)

###########
## Build ##
###########

include_directories(
include
${catkin_INCLUDE_DIRS}
${RBDL_INCLUDE_DIR}
${RBDL_URDFReader_INCLUDE_DIR}
${Eigen3_INCLUDE_DIRS}
${LOGGER_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(ROBOT_CLASS_SOURCE 
        src/robot.cpp 
        src/wheel_contact.cpp
        src/contact_v2.cpp 
        src/contacts.cpp 
        src/actuators.cpp 
        src/actuator.cpp 
        src/center_of_mass.cpp 
        src/state.cpp
)

## Declare a C++ library
 add_library(robot_class
   ${ROBOT_CLASS_SOURCE}
 )

 add_dependencies(robot_class ${catkin_EXPORTED_TARGETS} ${LOGGER_EXPORTED_TARGETS})

 target_link_libraries(robot_class
   ${catkin_LIBRARIES}
   ${RBDL_LIBRARY}
   ${RBDL_URDFReader_LIBRARY}
   ${LOGGER_LIBRARIES}
)

## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS robot_class
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME} include/mwoibn/communication_modules
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
