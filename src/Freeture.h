#pragma once
/*
*  
 * \author Andrea Novati - N3 S.r.l. 
 */

 //header refactoring ok
#include "Commons.h"

#include <boost/program_options.hpp>

namespace po        = boost::program_options;

namespace freeture
{
    class Device;

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
            void selectMode( po::variables_map&);

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
