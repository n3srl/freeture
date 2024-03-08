#pragma once
#define LOG_SPAM_FRAME_STATUS               false     // if true spam debug log every frame
#define DEFAULT_WAIT_TIME                   1000      // [ms]
#define METRICS_ACQUISITION_TIME            2         // [s]
#define DEFAULT_VIDEO_WAIT_BETWEEN_FRAMES   100       // [ms]
#define ARENA_TOOLS                         false     // include log print of all features values of the connected camera
#define USE_BOOST_LOG                       false     // true use boost log (faulty on multi-thread) false use log4cpp
#define FPS_ERROR_PERCENTAGE                0.03333333// [%] minimum FPS level is calculated multiplying what user configured by this number. if framerate is below this value an error is logged
#define FPS_MINIMUM_PERCENTAGE              (1.0 - FPS_ERROR_PERCENTAGE)