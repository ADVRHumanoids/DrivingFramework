# Generated by CMake 3.5.1

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget WheeledController::WheeledController)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target WheeledController::WheeledController
add_library(WheeledController::WheeledController SHARED IMPORTED)

set_target_properties(WheeledController::WheeledController PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/user/malgorzata/workspace/src/controllers/rt_plugins/src"
)

# Import target "WheeledController::WheeledController" for configuration ""
set_property(TARGET WheeledController::WheeledController APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(WheeledController::WheeledController PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "/home/user/malgorzata/workspace/devel_debug/.private/robot_class_xbot/lib/librobot_class_xbot.so;/home/user/advr-superbuild/build/install/lib/libXBotInterface.so.0.1.0;/home/user/advr-superbuild/build/install/lib/libXCM.so.0.1.0;/home/user/malgorzata/workspace/devel_debug/.private/robot_class_ros_extension/lib/librobot_class_ros_extension.so;/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_xbot/lib/libcommunication_modules_xbot.so;/home/user/advr-superbuild/build/install/lib/libXBotInterface.so;/home/user/malgorzata/workspace/devel_debug/.private/communication_modules_ros_extension/lib/libcommunication_modules_ros.so;/home/user/malgorzata/workspace/devel_debug/.private/custom_controller/lib/libcustom_controller.so;/opt/ros/kinetic/lib/libclass_loader.so;/usr/lib/libPocoFoundation.so;/usr/lib/x86_64-linux-gnu/libdl.so;/opt/ros/kinetic/lib/libroslib.so;/opt/ros/kinetic/lib/librospack.so;/usr/lib/x86_64-linux-gnu/libpython2.7.so;/usr/lib/x86_64-linux-gnu/libboost_program_options.so;/usr/lib/x86_64-linux-gnu/libtinyxml.so;/home/user/malgorzata/workspace/devel_debug/.private/controllers/lib/libwheeled_motion.so;/home/user/malgorzata/workspace/devel_debug/.private/controllers/lib/libcentralized_controllers.so;/home/user/malgorzata/workspace/devel_debug/.private/hierarchical_control/lib/libhierarchical_controller.so;/home/user/malgorzata/workspace/devel_debug/.private/hierarchical_control/lib/libcontroller_tasks.so;/home/user/malgorzata/workspace/devel_debug/.private/collision_model/lib/librobot_collision.so;/home/user/malgorzata/third_party/lib/lib/libsch-core.so;/home/user/malgorzata/workspace/devel_debug/lib/libpoints_handler.so;/home/user/malgorzata/workspace/devel_debug/.private/motor_side_reference/lib/libmotor_side_reference.so;/home/user/malgorzata/workspace/devel_debug/.private/robot_class/lib/librobot_class.so;/home/user/malgorzata/workspace/devel_debug/.private/point_handling/lib/libpoints_handler.so;/opt/ros/kinetic/lib/liburdf.so;/usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so;/usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so;/usr/lib/x86_64-linux-gnu/liburdfdom_model.so;/usr/lib/x86_64-linux-gnu/liburdfdom_world.so;/opt/ros/kinetic/lib/librosconsole_bridge.so;/opt/ros/kinetic/lib/libroscpp.so;/usr/lib/x86_64-linux-gnu/libboost_signals.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/opt/ros/kinetic/lib/librosconsole.so;/opt/ros/kinetic/lib/librosconsole_log4cxx.so;/opt/ros/kinetic/lib/librosconsole_backend_interface.so;/usr/lib/x86_64-linux-gnu/liblog4cxx.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/opt/ros/kinetic/lib/libroscpp_serialization.so;/opt/ros/kinetic/lib/librostime.so;/opt/ros/kinetic/lib/libxmlrpcpp.so;/opt/ros/kinetic/lib/libcpp_common.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/opt/ros/kinetic/lib/libsrdfdom.so;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so;/home/user/malgorzata/workspace/devel_debug/.private/eigen_utils/lib/libeigen_utils.so;/usr/lib/x86_64-linux-gnu/libyaml-cpp.so;/home/user/advr-superbuild/build/install/lib/librbdl.so;/home/user/advr-superbuild/build/install/lib/librbdl_urdfreader.so"
  IMPORTED_LOCATION_NOCONFIG "/home/user/malgorzata/workspace/src/controllers/rt_plugins/build/lib/libWheeledController.so.1.0.0"
  IMPORTED_SONAME_NOCONFIG "libWheeledController.so.1.0.0"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
