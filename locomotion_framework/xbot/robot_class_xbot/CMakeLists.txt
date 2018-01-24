cmake_minimum_required(VERSION 2.8.3)
project(robot_class_xbot)

find_package(catkin REQUIRED)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(LOG_INFO ${PROJECT_NAME})

# set C++11
add_definitions(-std=c++11)

# set-up options
option(LOGGER "Run loggers" OFF)

# variables for set-up options
set(LOGGER_INCLUDE_DIRS "")
set(LOGGER_LIBRARIES "")
set(LOGGER_EXPORTED_TARGETS "")

set(INSTALL_TARGETS "")

# LOAD LOGGER
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


# LOAD BASIC VARIABLES

find_package(catkin REQUIRED robot_class communication_modules_xbot communication_modules_ros_extension robot_class_ros_extension)

find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(XCM REQUIRED)

LIST(APPEND ROBOT_XBOT_LIBRARIES
	${urdf_LIBRARIES}
	${srdfdom_LIBRARIES}
	${XBotInterface_LIBRARIES}
	${XCM_LIBRARIES})

LIST(APPEND ROBOT_XBOT_INCLUDE_DIRS 
	${urdf_INCLUDE_DIRS}
	${srdfdom_INCLUDE_DIRS}
	${XBotInterface_INCLUDE_DIRS}
	${XCM_INCLUDE_DIRS}
)

LIST(APPEND ROBOT_XBOT_EXPORTED_TARGETS 
	${urdf_EXPORTED_TARGETS}
	${srdfdom_EXPORTED_TARGETS}
	${XBotInterface_EXPORTED_TARGETS}
	${XCM_EXPORTED_TARGETS}
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config)

catkin_package(
        INCLUDE_DIRS 
        	include 
        	${catkin_INCLUDE_DIRS} 
        	${ROBOT_XBOT_INCLUDE_DIRS}
        	${LOGGER_INCLUDE_DIRS}
        LIBRARIES 
        	${PROJECT_NAME}
        	${ROBOT_XBOT_LIBRARIES}
        	${LOGGER_LIBRARIES}
                ${catkin_LIBRARIES}
        CATKIN_DEPENDS
        	robot_class
        	robot_class_ros_extension
                communication_modules_xbot
)

###########
## Build ##
###########

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${ROBOT_XBOT_INCLUDE_DIRS}
        ${LOGGER_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(ROBOT_CLASS_SOURCE "")

#if(ROS)
	LIST(APPEND ROBOT_CLASS_SOURCE src/robot_xbot.cpp)
#endif(ROS)
#if(ROS_NRT)
	LIST(APPEND ROBOT_CLASS_SOURCE src/robot_xbot_feedback.cpp)
#endif(ROS_NRT)
	LIST(APPEND ROBOT_CLASS_SOURCE src/robot_xbot_nrt.cpp)
	LIST(APPEND ROBOT_CLASS_SOURCE src/robot_xbot_rt.cpp)
## Declare a C++ library
add_library(robot_class_xbot
        ${ROBOT_CLASS_SOURCE}
)

#	LIST(APPEND ROBOT_CLASS_SOURCE src/contact_ros.cpp)

add_dependencies(robot_class_xbot ${catkin_EXPORTED_TARGETS}
        ${ROBOT_XBOT_EXPORTED_TARGETS}# ${ROBOT_ROS_NRT_EXPORTED_TARGETS}
        ${LOGGER_EXPORTED_TARGETS}
)

target_link_libraries(robot_class_xbot
        ${catkin_LIBRARIES}
        ${ROBOT_XBOT_LIBRARIES} 
        #${ROBOT_ROS_NRT_LIBRARIES}
        ${LOGGER_LIBRARIES}
)


LIST(APPEND INSTALL_TARGETS 
	robot_class_xbot
)

# Mark executables and/or libraries for installation
install(TARGETS ${INSTALL_TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# Mark cpp header files for installation
 install(DIRECTORY include/mwoibn/robot_class
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mwoibn/
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY include/configs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include
  PATTERN ".svn" EXCLUDE)
