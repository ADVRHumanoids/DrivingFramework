cmake_minimum_required(VERSION 2.8.3)
project(filters)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

option(LOGGER OFF)
add_definitions(-std=c++11)

set(CATKIN_DEPENDS_SET "")
set(INSTALL_TARGETS "")

if(LOGGER)

LIST(APPEND CATKIN_DEPENDS_SET 
	simple_log
    )

endif(LOGGER)

LIST(APPEND CATKIN_DEPENDS_SET common)
LIST( APPEND INSTALL_TARGETS filters)
find_package(catkin REQUIRED ${CATKIN_DEPENDS_SET}) 

catkin_package(
  INCLUDE_DIRS 
	include 
	${catkin_INCLUDE_DIRS} 
  LIBRARIES
        ${INSTALL_TARGETS}
	${catkin_LIBRARIES} 
  CATKIN_DEPENDS
        ${CATKIN_DEPENDS_SET}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(filters
     src/iir_second_order.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(filters ${catkin_EXPORTED_TARGETS})

target_link_libraries(filters
     ${catkin_LIBRARIES} 
)


## Mark executables and/or libraries for installation
 install(TARGETS ${INSTALL_TARGETS}
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

