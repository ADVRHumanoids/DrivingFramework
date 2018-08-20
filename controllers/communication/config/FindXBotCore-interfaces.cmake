# - Try to find XBotInterface
# Once done this will define
#  XBOTINTERFACE_FOUND - System has LibXml2
#  XBOTINTERFACE_INCLUDE_DIRS - The LibXml2 include directories
#  XBOTINTERFACE_LIBRARIES - The libraries needed to use LibXml2
#  XBOTINTERFACE_DEFINITIONS - Compiler switches required for using LibXml2

#find_package(XBotCore-interfaces)

find_path(XBotCore-interfaces_INCLUDE_DIRS XBotCore-interfaces/All.h
          PATH_SUFFIXES include
          PATHS
          /usr/local/include/
          /usr/include/
          ${ADVR_SUPERBUILD_DIR}/build/install/include/)

# find the yaml-cpp library
#find_library(XBotCore-interfaces_LIBRARIES
#             NAMES XBotCore-interfaces
#             PATHS 
#                    /usr/lib/x86_64-linux-gnu/
#                    /usr/local
#                    /usr
#                    /sw
#                    /opt/local
#                    /opt/csw
#                    /opt
#                    ${ADVR_SUPERBUILD_DIR}/build/install/lib)

# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(XBotCore-interfaces DEFAULT_MSG XBotCore-interfaces_INCLUDE_DIRS)
mark_as_advanced(XBotCore-interfaces_INCLUDE_DIRS)
