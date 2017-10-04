# Searches for MatGeoLib includes and library files
#
# Sets the variables
#   RBDL_FOUND
#   RBDL_INCLUDE_DIR
#   RBDL_LIBRARY

SET (MathGeoLib_FOUND FALSE)

FIND_PATH (MathGeoLib_INCLUDE_DIRS MathGeoLib/MathGeoLib.h
	HINTS
	$ENV{HOME}/local/include
	$ENV{MathGeoLib_PATH}/src
	$ENV{MathGeoLib_PATH}/include
	$ENV{MathGeoLib_INCLUDE_PATH}
        /malgorzata/
	/usr/local/include
	/usr/include
        $ENV{HOME}/malgorzata/third_party/lib/include
	)

FIND_LIBRARY (MathGeoLib_LIBRARIES NAMES MathGeoLib
	PATHS
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{MathGeoLib_PATH}/lib
	$ENV{MathGeoLib_LIBRARY_PATH}
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	/usr/lib
	/usr/lib/x86_64-linux-gnu
        $ENV{HOME}/malgorzata/third_party/lib/lib
	)

IF (NOT MathGeoLib_LIBRARIES)
	MESSAGE (ERROR "Could not find MathGeoLib")
ENDIF (NOT MathGeoLib_LIBRARIES)

IF (MathGeoLib_INCLUDE_DIRS AND MathGeoLib_LIBRARIES)
	SET (MathGeoLib_FOUND TRUE)
ENDIF (MathGeoLib_INCLUDE_DIRS AND MathGeoLib_LIBRARIES)

MARK_AS_ADVANCED (
	MathGeoLib_INCLUDE_DIRS
	MathGeoLib_LIBRARIES
	)
