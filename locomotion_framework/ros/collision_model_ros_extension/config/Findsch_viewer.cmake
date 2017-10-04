# Searches for RBDL includes and library files, including Addons.
#
# Sets the variables
#   SCH_VIEWER_FOUND
#   sch_viewer_INCLUDE_DIRS
#   sch_viewer_LIBRARIES
#
# You can use the following components:
#   LuaModel
#   URDFReader
# and then link to them e.g. using RBDL_LuaModel_LIBRARY.

SET (SCH_VIEWER_FOUND FALSE)

FIND_PATH (sch_viewer_INCLUDE_DIRS sch/view/S_Object_gl.h
	HINTS
	$ENV{HOME}/local/include
	$ENV{SCH_VIEWER_PATH}/src
	$ENV{SCH_VIEWER_PATH}/include
	$ENV{SCH_VIEWER_INCLUDE_PATH}
	/usr/local/include
	/usr/include
	)

FIND_LIBRARY (sch_viewer_LIBRARIES NAMES sch_viewer
	PATHS
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{RBDL_PATH}/lib
	$ENV{RBDL_LIBRARY_PATH}
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	)

IF (NOT sch_viewer_LIBRARIES)
	MESSAGE (SEND_ERROR "sch_viewer_LIBRARIES ${sch_viewer_LIBRARIES}")
	MESSAGE (SEND_ERROR "sch_viewer_INCLUDE_DIRS ${sch_viewer_INCLUDE_DIRS}")
	MESSAGE (FATAL_ERROR "Could not find SCH_VIEWER")
ENDIF (NOT sch_viewer_LIBRARIES)

IF (sch_viewer_INCLUDE_DIRS AND sch_viewer_LIBRARIES)
	SET (SCH_VIEWER_FOUND TRUE)
ENDIF (sch_viewer_INCLUDE_DIRS AND sch_viewer_LIBRARIES)

IF (SCH_VIEWER_FOUND)
   IF (NOT SCH_VIEWER_FIND_QUIETLY)
      MESSAGE(STATUS "Found SCH_VIEWER: ${sch_viewer_LIBRARIES}")
   ENDIF (NOT SCH_VIEWER_FIND_QUIETLY)

ELSE (SCH_VIEWER_FOUND)
   IF (SCH_VIEWER_FIND_REQUIRED)
		 MESSAGE(FATAL_ERROR "Could not find SCH_VIEWER")
   ENDIF (SCH_VIEWER_FIND_REQUIRED)
ENDIF (SCH_VIEWER_FOUND)

MARK_AS_ADVANCED (
	sch_viewer_INCLUDE_DIRS
	sch_viewer_LIBRARIES
	)
