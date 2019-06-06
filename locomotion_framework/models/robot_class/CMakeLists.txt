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

find_package(catkin REQUIRED point_handling urdf srdfdom common eigen_utils filters)

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
find_package(YamlCpp REQUIRED)

catkin_package(
  INCLUDE_DIRS
			include
			${LOGGER_INCLUDE_DIRS}
            ${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include
  LIBRARIES
			robot_class
			communication
			${LOGGER_LIBRARIES}
	CATKIN_DEPENDS
			point_handling urdf srdfdom common eigen_utils filters
  DEPENDS
        YamlCpp

)

###########
## Build ##
###########

MESSAGE(WARNING "source path ${PROJECT_SOURCE_DIR}/../third_party/range-v3/include")

include_directories(
include
${PROJECT_SOURCE_DIR}/../../third_party/range-v3/include

${catkin_INCLUDE_DIRS}
${LOGGER_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()


## Declare a C++ library
add_library(communication
src/communication_modules/shared_controller.cpp
src/communication_modules/shared_point_setter.cpp )

add_dependencies(communication ${catkin_EXPORTED_TARGETS} ${LOGGER_EXPORTED_TARGETS})

target_link_libraries(communication
	${catkin_LIBRARIES}
	${LOGGER_LIBRARIES}
)


 add_library(robot_class
                 src/robot.cpp
                 src/robot_points/wheel_contact.cpp
                 src/robot_points/wheel_contact_v3.cpp
                 src/robot_points/contact_v2.cpp
                 src/robot_points/contact.cpp
                 src/robot_points/handler.cpp
                 src/robot_points/center_of_mass.cpp
                 src/robot_points/center_of_pressure.cpp
								 src/robot_points/point.cpp
								 src/robot_points/rotation.cpp
                # src/robot_points/center_of_pressure_fast.cpp
                 src/robot_points/torus_model.cpp
                 src/contacts.cpp
                 src/actuators.cpp
                 src/actuator.cpp )

 add_dependencies(robot_class ${catkin_EXPORTED_TARGETS} ${LOGGER_EXPORTED_TARGETS})
 target_compile_definitions(robot_class PUBLIC RBDL_BUILD_ADDON_URDFREADER)
 target_link_libraries(robot_class
# 	 communication
   ${catkin_LIBRARIES}
   ${LOGGER_LIBRARIES}
)

## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS robot_class communication
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/${PROJECT_NAME} include/mwoibn/communication_modules include/mwoibn/robot_points
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
