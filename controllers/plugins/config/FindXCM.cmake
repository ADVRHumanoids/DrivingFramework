# - Try to find XBotInterface
# Once done this will define
#  XBOTINTERFACE_FOUND - System has LibXml2
#  XBOTINTERFACE_INCLUDE_DIRS - The LibXml2 include directories
#  XBOTINTERFACE_LIBRARIES - The libraries needed to use LibXml2
#  XBOTINTERFACE_DEFINITIONS - Compiler switches required for using LibXml2

#find_package(XBotCore-interfaces)

find_path(XCM_INCLUDE_DIRS XCM/XBotControlPlugin.h
          PATH_SUFFIXES include
          PATHS
          /usr/local/include/
          /usr/include/
          ${ADVR_SUPERBUILD_DIR}/advr-superbuild/build/install/include/)

# find the yaml-cpp library
find_library(XCM_LIBRARIES
             NAMES XCM
             PATHS 
                    /usr/lib/x86_64-linux-gnu/
                    /usr/local
                    /usr
                    /sw
                    /opt/local
                    /opt/csw
                    /opt
                    ${ADVR_SUPERBUILD_DIR}/advr-superbuild/build/install/lib)

# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(XCM DEFAULT_MSG XCM_INCLUDE_DIRS)
mark_as_advanced(XCM_INCLUDE_DIRS)
