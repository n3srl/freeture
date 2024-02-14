#pragma once
/*
*  
 * \author Andrea Novati - N3 S.r.l. 
 */

 //header refactoring ok
#include <memory>

#include "Commons.h"
#include "FreetureSettings.h"


#include <boost/program_options.hpp>

namespace po        = boost::program_options;

namespace freeture
{
    class CameraDeviceManager;
    class CfgParam;
    class AcqThread;
    class DetThread;
    class StackThread;

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
            freeture::Mode m_CurrentRunMode = Mode::UNKNOWN;
            const char** m_Argv = nullptr;
            int m_Argc = -1;
            bool m_SigTermFlag = false;
            std::string      m_Version = std::string(VERSION);

            FreetureSettings m_FreetureCommandLineSettings;

            std::shared_ptr<CameraDeviceManager> m_CameraDeviceManager;
            std::shared_ptr<CfgParam> m_FreetureSettings;
            std::shared_ptr<AcqThread> m_AcquisitionThread;
            std::shared_ptr<DetThread> m_DetectionThread;
            std::shared_ptr<StackThread> m_StackThread;

            po::options_description* desc = nullptr;

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
