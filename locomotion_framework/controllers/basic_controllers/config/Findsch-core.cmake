# Searches for RBDL includes and library files, including Addons.
#
# Sets the variables
#   SCH-CORE_FOUND
#   SCH-CORE_INCLUDE_DIR
#   SCH-CORE_LIBRARY
#
# You can use the following components:
#   LuaModel
#   URDFReader
# and then link to them e.g. using RBDL_LuaModel_LIBRARY.

SET (SCH-CORE_FOUND FALSE)

FIND_PATH (SCH-CORE_INCLUDE_DIR sch/sch_api.h
	HINTS
	$ENV{HOME}/local/include
	$ENV{SCH-CORE_PATH}/src
	$ENV{SCH-CORE_PATH}/include
	$ENV{SCH-CORE_INCLUDE_PATH}
	/usr/local/include
	/usr/include
	)

FIND_LIBRARY (SCH-CORE_LIBRARY NAMES sch-core
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

IF (NOT SCH-CORE_LIBRARY)
	MESSAGE (ERROR "Could not find sch-core")
ENDIF (NOT SCH-CORE_LIBRARY)

IF (SCH-CORE_INCLUDE_DIR AND SCH-CORE_LIBRARY)
	SET (SCH-CORE_FOUND TRUE)
ENDIF (SCH-CORE_INCLUDE_DIR AND SCH-CORE_LIBRARY)

IF (SCH-CORE_FOUND)
   IF (NOT SCH-CORE_FIND_QUIETLY)
      MESSAGE(STATUS "Found SCH-CORE: ${SCH-CORE_LIBRARY}")
   ENDIF (NOT SCH-CORE_FIND_QUIETLY)

ELSE (SCH-CORE_FOUND)
   IF (SCH-CORE_FIND_REQUIRED)
		 MESSAGE(SEND_ERROR "Could not find SCH-CORE")
   ENDIF (SCH-CORE_FIND_REQUIRED)
ENDIF (SCH-CORE_FOUND)

MARK_AS_ADVANCED (
	SCH-CORE_INCLUDE_DIR
	SCH-CORE_LIBRARY
	)
