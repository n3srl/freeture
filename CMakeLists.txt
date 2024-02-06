# configure minimum cmake version
cmake_minimum_required(VERSION 3.10)

# configure project name and version
project(freeture VERSION 1.4)

#set cpp standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# Set the output directory for generated solution files (Visual Studio)
set(BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build" CACHE PATH "Build directory")

message(STATUS "Build directory: ${BUILD_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BUILD_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${BUILD_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BUILD_DIR}/lib")
set(CMAKE_BINARY_DIR "${BUILD_DIR}/prj")
#include config.h
set(CMAKE_INCLUDE_CURRENT_DIR ON)


message(STATUS "Runtime output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "Archive output directory: ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
message(STATUS "Library output directory: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "Binary directory: ${CMAKE_BINARY_DIR}")


# boost section
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "C:/Program Files/boost/boost_1_82_0/boost")

#set(Boost_USE_STATIC_LIBS OFF) 
#set(Boost_USE_MULTITHREADED ON)  
#set(Boost_USE_STATIC_RUNTIME OFF) 

find_package(Boost REQUIRED) 


if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
    add_definitions( "-DHAS_BOOST" )
endif()


#OpenCV Section
set(OpenCV_DIR "C:/Program Files/opencv/build")
								 
find_package(OpenCV REQUIRED) 



configure_file(config.h.in config.h COPYONLY)

#adding sources
set(SOURCES
	src/AcqSchedule.cpp
	src/AcqSchedule.h
	src/AcqThread.cpp
	src/AcqThread.h
	src/Base64.cpp
	src/Base64.h
	src/Camera.h
	src/CameraDeviceManager.cpp
	src/CameraDeviceManager.h
	src/CameraFirstInit.h
	src/CameraFrames.cpp
	src/CameraFrames.h
	src/CameraGigeAravis.cpp
	src/CameraGigeAravis.h
	src/CameraGigeAravisScanner.cpp
	src/CameraGigeAravisScanner.h
	src/CameraGigePylon.cpp
	src/CameraGigePylon.h
	src/CameraGigeTis.cpp
	src/CameraGigeTis.h
	src/CameraLucidArena.cpp
	src/CameraLucidArena.h
	src/CameraLucidArenaScanner.cpp
	src/CameraLucidArenaScanner.h
	src/CameraLucidArena_PHX016S.cpp
	src/CameraLucidArena_PHX016S.h
	src/CameraLucidArena_PHX016SScanner.cpp
	src/CameraLucidArena_PHX016SScanner.h
	src/CameraScanner.cpp
	src/CameraScanner.h
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
	src/ECamPixFmt.h
	src/ECamSdkType.h
	src/EDetMeth.h
	src/EImgBitDepth.h
	src/EImgFormat.h
	src/EInputDeviceType.h
	src/ELogSeverityLevel.h
	src/EParser.cpp
	src/EParser.h
	src/Ephemeris.cpp
	src/Ephemeris.h
	src/ErrorManager.cpp
	src/ErrorManager.h
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
	src/GlobalEvent.cpp
	src/GlobalEvent.h
	src/Histogram.cpp
	src/Histogram.h
	src/HistogramGray.cpp
	src/HistogramGray.h
	src/HistogramRGB.cpp
	src/HistogramRGB.h
	src/ImgProcessing.cpp
	src/ImgProcessing.h
	src/LocalEvent.cpp
	src/LocalEvent.h
	src/Logger.h
	src/main.cpp
	src/Mask.cpp
	src/Mask.h
	src/NodeExporterMetrics.cpp
	src/NodeExporterMetrics.h
	src/OpenSSL.cpp
	src/OpenSSL.h
	src/PixFmtConv.cpp
	src/PixFmtConv.h
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


include_directories(freeture ${OpenCV_INCLUDE_DIRS} "libs/ArenaSDK/include/Arena" "libs/ArenaSDK/GenICam/library/CPP/include" "libs/ArenaSDK/include/Save" "C:/msys64/clang32/include/glib-2.0/include" "C:/msys64/clang32/include/glib-2.0" "C:/msys64/clang32/include/aravis-0.8")

add_executable(freeture ${SOURCES})


#openssl
set_property(TARGET freeture
    PROPERTY VS_PACKAGE_REFERENCES "openssl-vc142"
)

target_link_libraries (freeture  ${OpenCV_LIBS})
