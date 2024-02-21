# configure minimum cmake version
cmake_minimum_required(VERSION 3.10)


#Configure project
# configure project name and version
project(freeture VERSION 1.4)

#set cpp standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# The version number.
set(VERSION 1.4.1)
# Configuration file default path
set(CFG_PATH /usr/local/share/freeture/ )

#Configure CMAKE

#unset this to run cmake find_packages debug
#set(CMAKE_FIND_DEBUG_MODE TRUE) 

# Set the output directory for generated solution files (Visual Studio)
set(BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build" CACHE PATH "Build directory")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BUILD_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${BUILD_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BUILD_DIR}/lib")

set(CMAKE_BINARY_DIR "${BUILD_DIR}/prj")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

message(STATUS "CMake Configuration; Runtime output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "CMake Configuration; Archive output directory: ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
message(STATUS "CMake Configuration; Library output directory: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "CMake Configuration; Binary directory: ${CMAKE_BINARY_DIR}")
message(STATUS "CMake Configuration; Build directory: ${BUILD_DIR}")










#Platform detection

if ( "${CMAKE_HOST_SYSTEM_NAME}" MATCHES "Windows")
	OPTION(LINUX "Linux system" OFF)
	OPTION(WINDOWS "Windows system" ON)
	message(STATUS "Platform detection; Windows platform detected")
	add_definitions(-D_WIN32_WINNT="0x0A00")
ELSE("${CMAKE_HOST_SYSTEM_NAME}" MATCHES "Linux")
	OPTION(LINUX "Linux system" ON)
	OPTION(WINDOWS "Windows system" OFF)
endif()

message(STATUS "Platform detection; CMAKE_HOST_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_NAME}")
message(STATUS "Platform detection; CMAKE_HOST_SYSTEM_VERSION ${CMAKE_HOST_SYSTEM_VERSION}")
message(STATUS "Platform detection; CMAKE_GENERATOR_TOOLSET ${CMAKE_GENERATOR_TOOLSET}")
message(STATUS "Platform detection; CMAKE_GENERATOR_PLATFORM ${CMAKE_GENERATOR_PLATFORM}")
message(STATUS "Platform detection; CMAKE_GENERATOR_INSTANCE ${CMAKE_GENERATOR_INSTANCE}")
message(STATUS "Platform detection; CMAKE_GENERATOR_INSTANCE ${CMAKE_GENERATOR}")







# boost section
#set (Boost_USE_STATIC_LIBS ON) 
set (Boost_USE_MULTITHREADED ON)  
set (Boost_USE_STATIC_LIBS ON)

#set( CMAKE_PREFIX_PATH 		"$ENV{Boost_DIR}/lib64-msvc-14.3/cmake/Boost-1.82.0;$ENV{Boost_DIR}/lib64-msvc-14.1/cmake/Boost-1.82.0;$ENV{Boost_DIR}/lib64-msvc-14.2/cmake/Boost-1.82.0")

message (STATUS "Boost Configuration; Boost_DIR:  $ENV{Boost_DIR}" )
message (STATUS "Boost Configuration; CMAKE_PREFIX_PATH:  ${CMAKE_PREFIX_PATH}" )


find_package(Boost 1.82.0 REQUIRED COMPONENTS system thread filesystem log program_options) 


message (STATUS "Boost Configuration; Boost_INCLUDE_DIRS:  ${Boost_INCLUDE_DIRS}" )
message (STATUS "Boost Configuration; Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}" )
message (STATUS "Boost Configuration; Boost_LIBRARIES: ${Boost_LIBRARIES}" )
message (STATUS "Boost Configuration; Boost_INCLUDES: ${Boost_INCLUDES}" )






#OpenCV Section
set(OpenCV_DIR "C:/local/OpenCV/build/")
							 
find_package(OpenCV REQUIRED) 

if (OpenCV_FOUND)
message (STATUS "OpenCV Configuration;Include OpenCV in config" )
	file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/config.h.in "#define USE_OPENCV true")
endif()





#LUCID ArenaSDK
set (Lucid_DIR "C:/Program Files/Lucid Vision Labs/Arena SDK")
				
set (Lucid_RELEASE "C:/Program Files/Lucid Vision Labs/Arena SDK/x64Release")
set (Lucid_Arena_INCLUDE "${Lucid_DIR}/include/Arena")
set (Lucid_Save_INCLUDE "${Lucid_DIR}/include/Save")
set (Lucid_GenICam_INCLUDE "${Lucid_DIR}/GenICam/library/CPP/include")

set (Lucid_INCLUDE_DIRS  
	${Lucid_Arena_INCLUDE}
	${Lucid_Save_INCLUDE}
	${Lucid_GenICam_INCLUDE}
)

set (Lucid_GenICam_LIBARARY "${Lucid_DIR}/GenICam/library/CPP/lib/Win64_x64")
										  
set (Lucid_GenICam_LIBARARIES 
 "${Lucid_GenICam_LIBARARY}/GCBase_MDd_VC140_v3_3_LUCID.lib"
 #"${Lucid_GenICam_LIBARARY}/GCBase_MD_VC140_v3_3_LUCID.lib"
 "${Lucid_GenICam_LIBARARY}/GenApi_MDd_VC140_v3_3_LUCID.lib"
 #"${Lucid_GenICam_LIBARARY}/GenApi_MD_VC140_v3_3_LUCID.lib"
 "${Lucid_GenICam_LIBARARY}/Log_MDd_VC140_v3_3_LUCID.lib"
 #"${Lucid_GenICam_LIBARARY}/Log_MD_VC140_v3_3_LUCID.lib"
 "${Lucid_GenICam_LIBARARY}/MathParser_MDd_VC140_v3_3_LUCID.lib"
 #"${Lucid_GenICam_LIBARARY}/MathParser_MD_VC140_v3_3_LUCID.lib"
 "${Lucid_GenICam_LIBARARY}/NodeMapData_MDd_VC140_v3_3_LUCID.lib"
 #"${Lucid_GenICam_LIBARARY}/NodeMapData_MD_VC140_v3_3_LUCID.lib"
 "${Lucid_GenICam_LIBARARY}/ResUsageStat_MDd_VC140_v3_3_LUCID.lib"
 #"${Lucid_GenICam_LIBARARY}/ResUsageStat_MD_VC140_v3_3_LUCID.lib"
 "${Lucid_GenICam_LIBARARY}/XmlParser_MDd_VC140_v3_3_LUCID.lib"
 #"${Lucid_GenICam_LIBARARY}/XmlParser_MD_VC140_v3_3_LUCID.lib"
)

set (Arena_FOUND true)

foreach(file  ${Lucid_GenICam_LIBARARIES})
	if ( NOT EXISTS "${file}")
		message (ERROR "Arena SDK Configuration;Lucid Library missing : ${file}" )
		set (Arena_FOUND false)
	endif()
endforeach()

if (Arena_FOUND)
	message (STATUS "Arena SDK Configuration;Include ARENA SDK in config" )
	file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/config.h.in "#define USE_ARENA true")
endif()

set(Lucid_ARENA_LIBRARIES
"${Lucid_DIR}/lib64/Arena/Arenad_v140.lib"
"${Lucid_DIR}/lib64/Arena/ArenaUId_v140.lib"
"${Lucid_DIR}/lib64/Arena/ArenaUI_v140.lib"
"${Lucid_DIR}/lib64/Arena/Arena_v140.lib"
"${Lucid_DIR}/lib64/Arena/GenTL_LUCIDd_v140.lib"
"${Lucid_DIR}/lib64/Arena/GenTL_LUCID_v140.lib"
"${Lucid_DIR}/lib64/Arena/lucidlogd_v140.lib"
"${Lucid_DIR}/lib64/Arena/lucidlog_v140.lib"
"${Lucid_DIR}/lib64/Arena/Saved_v140.lib"
"${Lucid_DIR}/lib64/Arena/Save_v140.lib"
)

set(Lucid_DLL 
#"${Lucid_RELEASE}/ArenaC_v140.dll"
#"${Lucid_RELEASE}/ArenaNET_v140.dll"
#"${Lucid_RELEASE}/ArenaUI_v140.dll"
#"${Lucid_RELEASE}/Arena_v140.dll"
#"${Lucid_RELEASE}/avcodec-58.dll"
#"${Lucid_RELEASE}/avformat-58.dll"
#"${Lucid_RELEASE}/avutil-56.dll"
#"${Lucid_RELEASE}/chrome_elf.dll"
#"${Lucid_RELEASE}/CLAllSerial_MD_VC140_v3_3_LUCID.dll"
#"${Lucid_RELEASE}/CLProtocol_MD_VC140_v3_3_LUCID.dll"
#"${Lucid_RELEASE}/d3dcompiler_43.dll"
#"${Lucid_RELEASE}/d3dcompiler_47.dll"
#"${Lucid_RELEASE}/FreeImage_v140.dll"
#"${Lucid_RELEASE}/GCBase_MD_VC140_v3_3_LUCID.dll"
#"${Lucid_RELEASE}/GenApi_MD_VC140_v3_3_LUCID.dll"
#"${Lucid_RELEASE}/GenTL_LUCID_v140.dll"
#"${Lucid_RELEASE}/libcef.dll"
#"${Lucid_RELEASE}/libEGL.dll"
#"${Lucid_RELEASE}/libGLESv2.dll"
#"${Lucid_RELEASE}/Log_MD_VC140_v3_3_LUCID.dll"
#"${Lucid_RELEASE}/lucidlog_v140.dll"
#"${Lucid_RELEASE}/MathParser_MD_VC140_v3_3_LUCID.dll"
#"${Lucid_RELEASE}/NodeMapData_MD_VC140_v3_3_LUCID.dll"
#"${Lucid_RELEASE}/ResUsageStat_MD_VC140_v3_3_LUCID.dll"
#"${Lucid_RELEASE}/rply.dll"
#"${Lucid_RELEASE}/SaveC_v140.dll"
#"${Lucid_RELEASE}/SaveNET_v140.dll"
#"${Lucid_RELEASE}/Save_v140.dll"
#"${Lucid_RELEASE}/swresample-3.dll"
#"${Lucid_RELEASE}/XmlParser_MD_VC140_v3_3_LUCID.dll"
)






#OpenSSL
set(OpenSSL_DIR "C:/Program Files/OpenSSL-Win64")
set(OpenSSL_INCLUDE_DIRS "${OpenSSL_DIR}/include")
set(OpenSSL_LIBARARY "${OpenSSL_DIR}/lib")

set (OpenSSL_LIBARARIES 
	"${OpenSSL_LIBARARY}/capi.lib"
	"${OpenSSL_LIBARARY}/dasync.lib"
	"${OpenSSL_LIBARARY}/libapps.lib"
	"${OpenSSL_LIBARARY}/libcrypto.lib"
	"${OpenSSL_LIBARARY}/libcrypto_static.lib"
	"${OpenSSL_LIBARARY}/libssl.lib"
	"${OpenSSL_LIBARARY}/libssl_static.lib"
	"${OpenSSL_LIBARARY}/libtestutil.lib"
	"${OpenSSL_LIBARARY}/openssl.lib"
	"${OpenSSL_LIBARARY}/ossltest.lib"
	"${OpenSSL_LIBARARY}/padlock.lib"
	"${OpenSSL_LIBARARY}/uitest.lib"
)







#CFITS
set(CFITS_DIR "C:/Program Files/cfits")
set(CFITS_INCLUDE_DIRS "${CFITS_DIR}/include")
set(CFITS_LIBARARY "${CFITS_DIR}/lib")

set (CFITS_LIBARARIES 
	"${CFITS_LIBARARY}/cfitsio.lib"
)









#adding sources

configure_file("./config.h.in" "${CMAKE_CURRENT_BINARY_DIR}/config.h")

set(SOURCES
	src/AcqSchedule.cpp
	src/AcqSchedule.h
	src/AcqThread.cpp
	src/AcqThread.h
	src/ArenaSDKManager.cpp
	src/ArenaSDKManager.h
	src/Base64.cpp
	src/Base64.h
	src/Camera.h
	src/CameraDeviceManager.cpp
	src/CameraDeviceManager.h
	src/CameraFactory.cpp
	src/CameraFactory.h
	src/CameraFirstInit.h
	src/CameraFrames.cpp
	src/CameraFrames.h
	src/CameraGigePylon.cpp
	src/CameraGigePylon.h
	src/CameraGigeTis.cpp
	src/CameraGigeTis.h
	src/CameraGigeAravis.cpp
	src/CameraGigeAravis.h
	src/CameraLucidAravis.h
	src/CameraLucidAravis.cpp
	src/CameraLucidArena_PHX016S.cpp
	src/CameraLucidArena_PHX016S.h
	src/CameraLucidAravis_Scanner.h
	src/CameraLucidArena_Scanner.cpp
	src/CameraLucidArena_Scanner.h
	src/CameraGigeAravis_Scanner.h
	src/CameraGigeAravis_Scanner.cpp
	src/CameraScanner.cpp
	src/CameraScanner.h
	src/CameraSettings.h
	src/CameraV4l2.cpp
	src/CameraV4l2.h
	src/CameraVideo.cpp
	src/CameraVideo.h
	src/CameraWindows.cpp
	src/CameraWindows.h
	src/CfgLoader.cpp
	src/CfgLoader.h
	src/CfgParam.cpp
	src/CfgParam.h
	src/Circle.h
	src/CommonConfig.h
	src/Commons.h
	src/Constants.h
	src/Conversion.cpp
	src/Conversion.h
	src/Detection.h
	src/DetectionTemplate.cpp
	src/DetectionTemplate.h
	src/DetectionTemporal.cpp
	src/DetectionTemporal.h
	src/DetThread.cpp
	src/DetThread.h
	src/Device.cpp
	src/Device.h
	src/Doxyfile
	src/EAcquisitionMode.h
	src/ECamModel.h
	src/ECamPixFmt.h
	src/ECamSdkType.h
	src/EDetMeth.h
	src/EFreetureMode.h
	src/EImgBitDepth.h
	src/EImgFormat.h
	src/EInputDeviceType.h
	src/ELogLevel.h
	src/ELogSeverityLevel.h
	src/ELogThread.h
	src/EParser.cpp
	src/EParser.h
	src/Ephemeris.cpp
	src/Ephemeris.h
	src/ESmtpSecurity.h
	src/EStackMeth.h
	src/ETimeMode.h
	src/ExposureControl.cpp
	src/ExposureControl.h
	src/Fits.cpp
	src/Fits.h
	src/Fits2D.cpp
	src/Fits2D.h
	src/Fits3D.cpp
	src/Fits3D.h
	src/Frame.cpp
	src/Frame.h
	src/Freeture.cpp
	src/Freeture.h
	src/FreetureSettings.h
	src/GlobalEvent.cpp
	src/GlobalEvent.h
	src/Histogram.cpp
	src/Histogram.h
	src/HistogramGray.cpp
	src/HistogramGray.h
	src/HistogramRGB.cpp
	src/HistogramRGB.h
	src/IAcqThread.h
	src/ICameraFab.h
	src/ImgProcessing.cpp
	src/ImgProcessing.h
	src/LocalEvent.cpp
	src/LocalEvent.h
	src/Logger.cpp
	src/Logger.h
	src/Loggercpp.cpp
	src/main.cpp
	src/Mask.cpp
	src/Mask.h
	src/NodeExporterMetrics.cpp
	src/NodeExporterMetrics.h
	src/OpenSSL.cpp
	src/OpenSSL.h
	src/PixFmtConv.cpp
	src/PixFmtConv.h
	src/PlatformSwitches.h
	src/SaveImg.cpp
	src/SaveImg.h
	src/SMTPClient.cpp
	src/SMTPClient.h
	src/Socket.h
	src/SParam.h
	src/Stack.cpp
	src/Stack.h
	src/StackThread.cpp
	src/StackThread.h
	src/TimeDate.cpp
	src/TimeDate.h
)


include_directories(
	freeture 
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS} 
	${Lucid_INCLUDE_DIRS}
	${OpenSSL_INCLUDE_DIRS}
	${CFITS_INCLUDE_DIRS}
	)

add_executable(freeture ${SOURCES})

add_definitions(-D_WIN32_WINNT=0x0601)

#openssl
set_property(TARGET freeture
    PROPERTY VS_PACKAGE_REFERENCES "openssl-vc142"
)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/share/configuration.cfg
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		
target_link_libraries (freeture  
	${OpenCV_LIBS} 
	${Boost_LIBRARIES} 
	${Lucid_GenICam_LIBARARIES} 
	${Lucid_ARENA_LIBRARIES}
	${OpenSSL_LIBARARIES}
	${CFITS_LIBARARIES}
	${Lucid_DLL}
	)
