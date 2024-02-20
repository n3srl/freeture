#pragma once
/*
*  
 * \author Andrea Novati - N3 S.r.l. 
 */

 //header refactoring ok
#include <memory>
#include <thread>

#include "Commons.h"
#include "FreetureSettings.h"
#include "EFreetureMode.h"


#include <boost/program_options.hpp>
#include <boost/asio/signal_set.hpp>

namespace po = boost::program_options;

namespace freeture
{
    class CameraDeviceManager;
    class CfgParam;
    class AcqThread;
    class DetThread;
    class StackThread;
    class Device;
    class Logger;
    
    class Freeture
    {
        private:
            std::thread::id m_ThreadID;
            std::shared_ptr<Logger> m_Logger;

            freeture::FreetureMode m_CurrentRunMode = FreetureMode::UNKNOWN;
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

            Device* m_Device;

            po::options_description* m_OptionsDescription = nullptr;

            void createDeviceManager();

            void printHelp();
            void printVersion();

            void selectListDevices();
            void selectListFormats();
            void selectMode( po::variables_map&);

            void signalHandler( int signum );
            void handler( const boost::system::error_code&, int );

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
