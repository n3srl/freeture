# - Try to find ArenaSDK
# Once done this will define
#  ARENASDK_FOUND - System has ArenaSDK 
#  ARENASDK_INCLUDE_DIRS - The ArenaSDK  include directories
#  ARENASDK_LIBRARIES - The libraries needed to use ArenaSDK

IF(${OperatingSystem} MATCHES "Windows")
	IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
		set( ARENASDK_LIBRARY "$ENV{ARENASDK_ROOT}/lib/x64" )
		set( GENICAM_LIBRARY "$ENV{ARENASDK_ROOT}/lib/x64" )
		set( FFMPEG_LIBRARY "$ENV{ARENASDK_ROOT}/lib/x64" )
	ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
ELSE(${OperatingSystem} MATCHES "Windows")
	IF(${OperatingSystem} MATCHES "Linux")
		IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
			set( ARENASDK_LIBRARY "/opt/arenasdk/lib64" )
			set( GENICAM_ROOT_LIBRARY "/opt/arenasdk/GenICam/library/lib/Linux64_x64" )
			set( FFMPEG_ROOT_LIBRARY "/opt/arenasdk/ffmpeg" )
		ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
	ENDIF(${OperatingSystem} MATCHES "Linux")
ENDIF(${OperatingSystem} MATCHES "Windows")

FIND_PATH(	ARENAAPI_INCLUDE_DIR
			NAMES ArenaApi.h Arena/ArenaApi.h
			PATHS
			/opt/arenasdk/include/Arena
			"$ENV{ARENASDK_ROOT}/include/Arena"
)

FIND_PATH(	SAVEAPI_INCLUDE_DIR
			NAMES SaveApi.h Save/SaveApi.h 
			PATHS
			/opt/arenasdk/include/Save
			"$ENV{ARENASDK_ROOT}/include/Save"
)

FIND_PATH(	GENTL_INCLUDE_DIR
			NAMES GenTL.h GentTL/GenTL.h
			PATHS
			/opt/arenasdk/include/GenTL
			"$ENV{ARENASDK_ROOT}/include/GenTL"
)


FIND_PATH(	GENICAM_INCLUDE_DIR
			NAMES GenICam.h
			PATHS
			/opt/arenasdk/GenICam/library/CPP/include
			"$ENV{ARENASDK_ROOT}/include/GenTL"
)

FIND_LIBRARY(	GENICAM_LIBRARY 
				NAMES 
				libGCBase_gcc54_v3_3_LUCID.so
				liblog4cpp_gcc54_v3_3_LUCID.so
				libMathParser_gcc54_v3_3_LUCID.so
				libXmlParser_gcc54_v3_3_LUCID.so
				libGenApi_gcc54_v3_3_LUCID.so
				libLog_gcc54_v3_3_LUCID.so
				libNodeMapData_gcc54_v3_3_LUCID.so
				PATHS
				${GENICAM_ROOT_LIBRARY}
)

FIND_LIBRARY(	FFMPEG_LIBRARY 
				NAMES 
				libavcodec.so
				libavcodec.so.58
				libavcodec.so.58.18.100
				libavformat.so
				libavformat.so.58
				libavformat.so.58.12.100
				libavutil.so
				libavutil.so.56
				libavutil.so.56.14.100
				libswresample.so
				libswresample.so.3
				libswresample.so.3.1.100
				PATHS
				${FFMPEG_ROOT_LIBRARY}
)


FIND_LIBRARY(	ARENAAPI_LIBRARY 
				NAMES 
				libarena.so
				PATHS
				${ARENASDK_LIBRARY}
)

FIND_LIBRARY(	SAVEAPI_LIBRARY 
				NAMES 
				libsave.so
				PATHS
				${ARENASDK_LIBRARY}
)

FIND_LIBRARY(	GENTL_LIBRARY 
				NAMES 
				libgentl.so
				PATHS
				${ARENASDK_LIBRARY}
)

FIND_LIBRARY(	LUCIDLOG_LIBRARY 
				NAMES 
				loblucidlog.so
				PATHS
				${ARENASDK_LIBRARY}
)

if( NOT ARENAAPI_LIBRARY)
    set(ARENAAPI_LIBRARY "")
endif(NOT ARENAAPI_LIBRARY)

if( NOT SAVEAPI_LIBRARY)
    set(SAVEAPI_LIBRARY "")
endif(NOT SAVEAPI_LIBRARY)

if( NOT GENTL_LIBRARY)
    set(GENTL_LIBRARY "")
endif(NOT GENTL_LIBRARY)

if( NOT LUCIDLOG_LIBRARY)
    set(LUCIDLOG_LIBRARY "")
endif(NOT LUCIDLOG_LIBRARY)

set(ARENASDK_LIBRARIES  ${ARENAAPI_LIBRARY} ${SAVEAPI_LIBRARY} ${GENTL_LIBRARY} ${LUCIDLOG_LIBRARY} ${GENICAM_LIBRARY} )

set(ARENASDK_INCLUDE_DIR ${ARENAAPI_INCLUDE_DIR}  ${SAVEAPI_INCLUDE_DIR}  ${GENTL_INCLUDE_DIR} ${GENICAM_INCLUDE_DIR} )

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(ARENASDK DEFAULT_MSG
  ARENASDK_INCLUDE_DIR
  ARENASDK_LIBRARY)

mark_as_advanced(ARENASDK_INCLUDE_DIR ARENASDK_LIBRARIES)
