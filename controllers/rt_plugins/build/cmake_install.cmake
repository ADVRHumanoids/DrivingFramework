# Install script for directory: /home/user/malgorzata/workspace/src/controllers/rt_plugins

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/user/advr-superbuild/build/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/advr-superbuild/build/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/user/advr-superbuild/build/install" TYPE PROGRAM FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/advr-superbuild/build/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/user/advr-superbuild/build/install" TYPE PROGRAM FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/advr-superbuild/build/install/setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/user/advr-superbuild/build/install" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/catkin_generated/installspace/setup.bash")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/advr-superbuild/build/install/setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/user/advr-superbuild/build/install" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/catkin_generated/installspace/setup.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/advr-superbuild/build/install/setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/user/advr-superbuild/build/install" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/catkin_generated/installspace/setup.zsh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/advr-superbuild/build/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/user/advr-superbuild/build/install" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/include/" FILES_MATCHING REGEX "/[^/]*\\.h[^/]*$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "shlib")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRtMyTest.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRtMyTest.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/libRtMyTest.so.1.0.0"
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/libRtMyTest.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRtMyTest.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRtMyTest.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/user/malgorzata/workspace/devel_debug/.private/robot_class_xbot/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_xbot/lib:/home/user/advr-superbuild/build/install/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/custom_controller/lib:/opt/ros/kinetic/lib:/home/user/malgorzata/workspace/devel_debug/.private/controllers/lib:/home/user/malgorzata/workspace/devel_debug/.private/hierarchical_control/lib:/home/user/malgorzata/workspace/devel_debug/.private/collision_model/lib:/home/user/malgorzata/third_party/lib/lib:/home/user/malgorzata/workspace/devel_debug/lib:/home/user/malgorzata/workspace/devel_debug/.private/motor_side_reference/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class/lib:/home/user/malgorzata/workspace/devel_debug/.private/point_handling/lib:/home/user/malgorzata/workspace/devel_debug/.private/eigen_utils/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/RtMyTest" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/RtMyTestConfigVersion.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/RtMyTest" TYPE FILE RENAME "RtMyTestConfig.cmake" FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/RtMyTestConfig.cmake.install")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/RtMyTest/RtMyTestTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/RtMyTest/RtMyTestTargets.cmake"
         "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/RtMyTest/RtMyTestTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/RtMyTest/RtMyTestTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/RtMyTest/RtMyTestTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/RtMyTest" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/RtMyTest/RtMyTestTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/RtMyTest" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/RtMyTest/RtMyTestTargets-noconfig.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/include/" FILES_MATCHING REGEX "/[^/]*\\.h[^/]*$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "shlib")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCentralizedController.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCentralizedController.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libCentralizedController.so.1.0.0"
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libCentralizedController.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCentralizedController.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCentralizedController.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/user/malgorzata/workspace/devel_debug/.private/robot_class_xbot/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_xbot/lib:/home/user/advr-superbuild/build/install/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/custom_controller/lib:/opt/ros/kinetic/lib:/home/user/malgorzata/workspace/devel_debug/.private/controllers/lib:/home/user/malgorzata/workspace/devel_debug/.private/hierarchical_control/lib:/home/user/malgorzata/workspace/devel_debug/.private/collision_model/lib:/home/user/malgorzata/third_party/lib/lib:/home/user/malgorzata/workspace/devel_debug/lib:/home/user/malgorzata/workspace/devel_debug/.private/motor_side_reference/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class/lib:/home/user/malgorzata/workspace/devel_debug/.private/point_handling/lib:/home/user/malgorzata/workspace/devel_debug/.private/eigen_utils/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/CentralizedController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CentralizedControllerConfigVersion.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/CentralizedController" TYPE FILE RENAME "CentralizedControllerConfig.cmake" FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CentralizedControllerConfig.cmake.install")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/CentralizedController/CentralizedControllerTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/CentralizedController/CentralizedControllerTargets.cmake"
         "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/CentralizedController/CentralizedControllerTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/CentralizedController/CentralizedControllerTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/CentralizedController/CentralizedControllerTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/CentralizedController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/CentralizedController/CentralizedControllerTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/CentralizedController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/CentralizedController/CentralizedControllerTargets-noconfig.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/include/" FILES_MATCHING REGEX "/[^/]*\\.h[^/]*$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "shlib")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOnlineCentralizedControllerPlugin.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOnlineCentralizedControllerPlugin.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libOnlineCentralizedControllerPlugin.so.1.0.0"
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libOnlineCentralizedControllerPlugin.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOnlineCentralizedControllerPlugin.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOnlineCentralizedControllerPlugin.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/user/malgorzata/workspace/devel_debug/.private/robot_class_xbot/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_xbot/lib:/home/user/advr-superbuild/build/install/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/custom_controller/lib:/opt/ros/kinetic/lib:/home/user/malgorzata/workspace/devel_debug/.private/controllers/lib:/home/user/malgorzata/workspace/devel_debug/.private/hierarchical_control/lib:/home/user/malgorzata/workspace/devel_debug/.private/collision_model/lib:/home/user/malgorzata/third_party/lib/lib:/home/user/malgorzata/workspace/devel_debug/lib:/home/user/malgorzata/workspace/devel_debug/.private/motor_side_reference/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class/lib:/home/user/malgorzata/workspace/devel_debug/.private/point_handling/lib:/home/user/malgorzata/workspace/devel_debug/.private/eigen_utils/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/OnlineCentralizedControllerPlugin" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/OnlineCentralizedControllerPluginConfigVersion.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/OnlineCentralizedControllerPlugin" TYPE FILE RENAME "OnlineCentralizedControllerPluginConfig.cmake" FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/OnlineCentralizedControllerPluginConfig.cmake.install")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/OnlineCentralizedControllerPlugin/OnlineCentralizedControllerPluginTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/OnlineCentralizedControllerPlugin/OnlineCentralizedControllerPluginTargets.cmake"
         "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/OnlineCentralizedControllerPlugin/OnlineCentralizedControllerPluginTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/OnlineCentralizedControllerPlugin/OnlineCentralizedControllerPluginTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/OnlineCentralizedControllerPlugin/OnlineCentralizedControllerPluginTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/OnlineCentralizedControllerPlugin" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/OnlineCentralizedControllerPlugin/OnlineCentralizedControllerPluginTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/OnlineCentralizedControllerPlugin" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/OnlineCentralizedControllerPlugin/OnlineCentralizedControllerPluginTargets-noconfig.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/include/" FILES_MATCHING REGEX "/[^/]*\\.h[^/]*$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "shlib")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libGravityTest.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libGravityTest.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libGravityTest.so.1.0.0"
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libGravityTest.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libGravityTest.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libGravityTest.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/user/malgorzata/workspace/devel_debug/.private/robot_class_xbot/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_xbot/lib:/home/user/advr-superbuild/build/install/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/custom_controller/lib:/opt/ros/kinetic/lib:/home/user/malgorzata/workspace/devel_debug/.private/controllers/lib:/home/user/malgorzata/workspace/devel_debug/.private/hierarchical_control/lib:/home/user/malgorzata/workspace/devel_debug/.private/collision_model/lib:/home/user/malgorzata/third_party/lib/lib:/home/user/malgorzata/workspace/devel_debug/lib:/home/user/malgorzata/workspace/devel_debug/.private/motor_side_reference/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class/lib:/home/user/malgorzata/workspace/devel_debug/.private/point_handling/lib:/home/user/malgorzata/workspace/devel_debug/.private/eigen_utils/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/GravityTest" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/GravityTestConfigVersion.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/GravityTest" TYPE FILE RENAME "GravityTestConfig.cmake" FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/GravityTestConfig.cmake.install")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/GravityTest/GravityTestTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/GravityTest/GravityTestTargets.cmake"
         "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/GravityTest/GravityTestTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/GravityTest/GravityTestTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/GravityTest/GravityTestTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/GravityTest" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/GravityTest/GravityTestTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/GravityTest" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/GravityTest/GravityTestTargets-noconfig.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/include/" FILES_MATCHING REGEX "/[^/]*\\.h[^/]*$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "shlib")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libWheeledController.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libWheeledController.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libWheeledController.so.1.0.0"
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libWheeledController.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libWheeledController.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libWheeledController.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/user/malgorzata/workspace/devel_debug/.private/robot_class_xbot/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_xbot/lib:/home/user/advr-superbuild/build/install/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/custom_controller/lib:/opt/ros/kinetic/lib:/home/user/malgorzata/workspace/devel_debug/.private/controllers/lib:/home/user/malgorzata/workspace/devel_debug/.private/hierarchical_control/lib:/home/user/malgorzata/workspace/devel_debug/.private/collision_model/lib:/home/user/malgorzata/third_party/lib/lib:/home/user/malgorzata/workspace/devel_debug/lib:/home/user/malgorzata/workspace/devel_debug/.private/motor_side_reference/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class/lib:/home/user/malgorzata/workspace/devel_debug/.private/point_handling/lib:/home/user/malgorzata/workspace/devel_debug/.private/eigen_utils/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/WheeledController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/WheeledControllerConfigVersion.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/WheeledController" TYPE FILE RENAME "WheeledControllerConfig.cmake" FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/WheeledControllerConfig.cmake.install")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/WheeledController/WheeledControllerTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/WheeledController/WheeledControllerTargets.cmake"
         "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/WheeledController/WheeledControllerTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/WheeledController/WheeledControllerTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/WheeledController/WheeledControllerTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/WheeledController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/WheeledController/WheeledControllerTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/WheeledController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/WheeledController/WheeledControllerTargets-noconfig.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/include/" FILES_MATCHING REGEX "/[^/]*\\.h[^/]*$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "shlib")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCombinedController.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCombinedController.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libCombinedController.so.1.0.0"
    "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libCombinedController.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCombinedController.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCombinedController.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/user/malgorzata/workspace/devel_debug/.private/robot_class_xbot/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_xbot/lib:/home/user/advr-superbuild/build/install/lib:/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_ros_extension/lib:/home/user/malgorzata/workspace/devel_debug/.private/custom_controller/lib:/opt/ros/kinetic/lib:/home/user/malgorzata/workspace/devel_debug/.private/controllers/lib:/home/user/malgorzata/workspace/devel_debug/.private/hierarchical_control/lib:/home/user/malgorzata/workspace/devel_debug/.private/collision_model/lib:/home/user/malgorzata/third_party/lib/lib:/home/user/malgorzata/workspace/devel_debug/lib:/home/user/malgorzata/workspace/devel_debug/.private/motor_side_reference/lib:/home/user/malgorzata/workspace/devel_debug/.private/robot_class/lib:/home/user/malgorzata/workspace/devel_debug/.private/point_handling/lib:/home/user/malgorzata/workspace/devel_debug/.private/eigen_utils/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/CombinedController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CombinedControllerConfigVersion.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/CombinedController" TYPE FILE RENAME "CombinedControllerConfig.cmake" FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CombinedControllerConfig.cmake.install")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/CombinedController/CombinedControllerTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/CombinedController/CombinedControllerTargets.cmake"
         "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/CombinedController/CombinedControllerTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/CombinedController/CombinedControllerTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/CombinedController/CombinedControllerTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/CombinedController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/CombinedController/CombinedControllerTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/CombinedController" TYPE FILE FILES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/CMakeFiles/Export/lib/cmake/CombinedController/CombinedControllerTargets-noconfig.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
