#pragma once

#include "config.h"

#ifdef WINDOWS
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
    #include <process.h>
#else
    #ifdef LINUX
        #include <signal.h>
        #include <unistd.h>
        #include <termios.h>
        #include <sys/types.h>
        #include <sys/time.h>

        #define BOOST_LOG_DYN_LINK 1
    #endif
#endif

#include "Device.h"
#include "Frame.h"
#include "Histogram.h"
#include "HistogramGray.h"
#include "HistogramRGB.h"
#include "SaveImg.h"
#include "Conversion.h"
#include "Fits2D.h"
#include "EImgBitDepth.h"
#include "EParser.h"
#include "EDetMeth.h"
#include "DetThread.h"
#include "StackThread.h"
#include "AcqThread.h"
#include "CameraGigeTis.h"
#include "ImgProcessing.h"
#include "Logger.h"
#include "CameraWindows.h"
#include "ECamPixFmt.h"
#include "CfgParam.h"






#include <boost/program_options.hpp>

namespace po        = boost::program_options;
namespace logging   = boost::log;
namespace sinks     = boost::log::sinks;
namespace attrs     = boost::log::attributes;
namespace src       = boost::log::sources;
namespace expr      = boost::log::expressions;
namespace keywords  = boost::log::keywords;


namespace freeture
{

    enum class Mode
    {
        TEST_CONFIGURATION,
        PRINT_HELP,
        PRINT_VERSION,
        CONTINUOUS_ACQUISITION,
        METEOR_DETECTION,
        SINGLE_ACQUISITION,
        CLEAN_LOGS,
        LIST_DEVICES,
        LIST_FORMATS,
        UNKNOWN
    };

    class Freeture
    {
        private:
            po::options_description* desc = nullptr;
            bool sigTermFlag        = false;
            Device *device          = nullptr;
            int m_Argc              = -1;
            const char ** m_Argv    = nullptr;
            bool m_Error            = false;

            freeture::Mode        m_CurrentMode = Mode::UNKNOWN;

            int         mode            = -1;
            int         executionTime   = 0;
            std::string      configPath      = std::string(CFG_PATH) + "configuration.cfg";
            std::string      savePath        = "./";
            int         acqFormat       = 0;
            int         acqWidth        = 0;
            int         acqHeight       = 0;
            int         startx          = 0;
            int         starty          = 0;
            int         gain            = 0;
            double      exp             = 0;
            std::string      version         = std::string(VERSION);
            int         devID           = 0;
            std::string      fileName        = "snap";
            bool        listFormats     = false;
            bool        display         = false;
            bool        bmp             = false;
            bool        fits            = false;
            bool        sendbymail      = false;

            void printHelp();
            void printVersion();

            void selectListDevices();
            void selectListFormats();
            void selectMode( boost::program_options::variables_map&);

            void initLogger( std::string , LogSeverityLevel );

            void signalHandler( int signum );

            void modeTest();
            void modeContinuousAcquisition();
            void modeMeteorDetection();
            void modeSingleAcquisition();
            void modeCleanLogs();
            void fetchProgramOption();
        public:
            Freeture(int , const char** );

            void Run();
    };
}
