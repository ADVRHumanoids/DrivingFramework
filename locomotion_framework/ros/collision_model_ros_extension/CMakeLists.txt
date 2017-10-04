cmake_minimum_required(VERSION 2.8.3)
project(collision_model_ros_extension)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(VISUALIZATION_MODULE OFF)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
#  urdf
#  srdfdom
#  gazebo_msgs
  robot_class
  collision_model
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
set(DEPENDS_OPTIONAL "")
set(CATKIN_DEPENDS_OPTIONAL "")
if(VISUALIZATION_MODULE)
   find_package(sch_viewer REQUIRED)
   find_package(robot_class_ros_extension REQUIRED)

   include_directories(${sch_viewer_INCLUDE_DIRS} ${robot_class_ros_extension_INCLUDE_DIRS} )
   if(NOT SCH_VIEWER_FOUND)
     message(ERROR " SCH_VIEWER not found!")
   endif(NOT SCH_VIEWER_FOUND)

   find_package(GLUT REQUIRED)
   include_directories(${GLUT_INCLUDE_DIRS} )
   link_directories(${GLUT_LIBRARY_DIRS})
   add_definitions(${GLUT_DEFINITIONS})
   if(NOT GLUT_FOUND)
     message(ERROR " GLUT not found!")
   endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
   find_package(OpenGL REQUIRED)
   include_directories(${OpenGL_INCLUDE_DIRS})
   link_directories(${OpenGL_LIBRARY_DIRS})
   add_definitions(${OpenGL_DEFINITIONS})
   if(NOT OPENGL_FOUND)
      message(ERROR " OPENGL not found!")
   endif(NOT OPENGL_FOUND)

   LIST( APPEND DEPENDS_OPTIONAL sch_viewer)
   LIST( APPEND CATKIN_DEPENDS_OPTIONAL robot_class_ros_extension)
#  


endif() 

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
     roscpp   
     collision_model
#     urdf
#     srdfdom
     robot_class
     ${CATKIN_DEPENDS_OPTIONAL}
  DEPENDS 
     ${DEPENDS_OPTIONAL}
)
 
include_directories(
  include
  ${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

if(VISUALIZATION_MODULE)
 add_executable(vision src/vision.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(vision ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(vision
   ${catkin_LIBRARIES} 
   ${robot_class_ros_extension_LIBRARIES}
   ${sch_viewer_LIBRARIES}
   ${OPENGL_gl_LIBRARY}
   ${GLUT_LIBRARIES} 
 )
endif(VISUALIZATION_MODULE)


if(VISUALIZATION_MODULE)
 install(TARGETS vision
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
endif(VISUALIZATION_MODULE)

## Mark cpp header files for installation
 install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


#############
## Testing ##
#############


if(CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)
 find_package(tests_common REQUIRED)
 find_package(roscpp REQUIRED)
 find_package(centauro REQUIRED)
 find_package(robot_class_ros_extension REQUIRED)


 LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config )
 find_package(RBDL REQUIRED)
 find_package(Eigen3 REQUIRED)

 add_rostest_gtest(collision_model_ros_test test/launch/test_collision_model_ros.test test/src/test_collision_model_ros.cpp)

# catkin_add_gtest(${PROJECT_NAME}-test test/src/test_collision_model_ros.cpp)

 if(TARGET collision_model_ros_test)
  include_directories(${tests_common_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${centauro_INCLUDE_DIRS} ${robot_class_ros_extension_INCLUDE_DIRS})
  add_dependencies(collision_model_ros_test ${catkin_EXPORTED_TARGETS} ${tests_common_EXPORTED_TARGETS} ${roscpp_EXPORTED_TARGETS} ${centauro_EXPORTED_TARGETS} ${robot_class_ros_extension_EXPORTED_TARGETS})
  target_link_libraries(collision_model_ros_test
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY}
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${tests_common_LIBRARIES}
        ${centauro_LIBRARIES}
        ${robot_class_ros_extension_LIBRARIES}
   )
 endif()

endif(CATKIN_ENABLE_TESTING)
