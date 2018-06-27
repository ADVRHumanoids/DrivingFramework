# - Try to find XBotInterface
# Once done this will define
#  XBOTINTERFACE_FOUND - System has LibXml2
#  XBOTINTERFACE_INCLUDE_DIRS - The LibXml2 include directories
#  XBOTINTERFACE_LIBRARIES - The libraries needed to use LibXml2
#  XBOTINTERFACE_DEFINITIONS - Compiler switches required for using LibXml2

#find_package(XBotInterface)

find_path(XBotInterface_INCLUDE_DIRS XBotInterface/XBotInterface.h
          PATH_SUFFIXES include
          PATHS
          /usr/local/include/
          /usr/include/
          ${ADVR_SUPERBUILD_DIR}/build/install/include/)

# find the yaml-cpp library
find_library(XBotInterface_LIBRARIES
             NAMES XBotInterface
             PATHS 
                    /usr/lib/x86_64-linux-gnu/
                    /usr/local
                    /usr
                    /sw
                    /opt/local
                    /opt/csw
                    /opt
                    ${ADVR_SUPERBUILD_DIR}/build/install/lib)

# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(XBotInterface DEFAULT_MSG XBotInterface_INCLUDE_DIRS XBotInterface_LIBRARIES)
mark_as_advanced(XBotInterface_INCLUDE_DIRS XBotInterface_LIBRARIES)
