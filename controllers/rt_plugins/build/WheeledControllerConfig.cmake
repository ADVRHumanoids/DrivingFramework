set(WheeledController_VERSION 1.0.0)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was WheeledControllerConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

set_and_check(WheeledController_INCLUDEDIR "/home/user/malgorzata/workspace/src/controllers/rt_plugins/src")

if(NOT TARGET WheeledController::WheeledController)
  include("${CMAKE_CURRENT_LIST_DIR}/WheeledControllerTargets.cmake")
endif()

# Compatibility
set(WheeledController_LIBRARIES WheeledController::WheeledController)
set(WheeledController_INCLUDE_DIRS ${WheeledController_INCLUDEDIR})
