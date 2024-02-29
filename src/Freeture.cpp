//header refactoring ok
#include "Freeture.h"

#include "CameraDeviceManager.h"

#include "ECamPixFmt.h"
#include "EParser.h"
#include "Device.h"
#include "CfgParam.h"
#include "Frame.h"

#include "AcqThread.h"
#include "DetThread.h"
#include "StackThread.h"

#include "Logger.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/filesystem.hpp>

#include "Conversion.h"
#include "ImgProcessing.h"
#include "SaveImg.h"
#include "Fits2D.h"
#include "SMTPClient.h"

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace std;
using namespace freeture;

/// <summary>
/// Signal handling
/// </summary>
/// <param name="signum"></param>
void Freeture::signalHandler(int signum)
{
    LOG_WARNING << "Freeture::signalHandler;" << "Received signal : " << signum << endl;
    m_SigTermFlag = true;
}

void Freeture::handler(const boost::system::error_code& error, int signal_number)
{
    LOG_WARNING << "Freeture::signalHandler;" << "handling signal " << signal_number << endl;
    m_SigTermFlag = true;
}


freeture::Freeture::Freeture(int argc, const char** argv) :
    m_FreetureSettings(std::make_shared<CfgParam>(m_FreetureCommandLineSettings.configurationFilePath))
{
    m_Argc=argc;
    m_Argv=argv;
}

/// <summary>
/// Given variable map configure current object state for an operative mode
/// </summary>
/// <param name="vm"></param>
void Freeture::selectMode( boost::program_options::variables_map& vm)
{
    m_CurrentRunMode = FreetureMode::UNKNOWN;

    switch(m_FreetureCommandLineSettings.mode)
    {
        case 1:
        {
            m_CurrentRunMode = FreetureMode::TEST_CONFIGURATION;
        }
        break;
        case 2 :
        {
            m_CurrentRunMode = FreetureMode::CONTINUOUS_ACQUISITION;
        }
        break;
        case 3:
        {
            m_CurrentRunMode = FreetureMode::METEOR_DETECTION;
        }
        break;
        case 4 :
        {
            m_CurrentRunMode = FreetureMode::SINGLE_ACQUISITION;
        }
        break;
        case 5:
        {
            m_CurrentRunMode = FreetureMode::CLEAN_LOGS;
        }
        break;
    }

    if (
        m_CurrentRunMode == FreetureMode::METEOR_DETECTION ||
        m_CurrentRunMode == FreetureMode::SINGLE_ACQUISITION ||
        m_CurrentRunMode == FreetureMode::CONTINUOUS_ACQUISITION
    )
    {
        
            // Cam id.
            if(vm.count("id")) m_FreetureCommandLineSettings.devID = vm["id"].as<int>();

            // Path where to save files.
            if(vm.count("savepath")) m_FreetureCommandLineSettings.savePath = vm["savepath"].as<string>();

            // Acquisition pixel format.
            if(vm.count("format")) m_FreetureCommandLineSettings.acqFormat = vm["format"].as<int>();

            // Crop start x
            if(vm.count("startx")) m_FreetureCommandLineSettings.startx = vm["startx"].as<int>();

            // Crop start y
            if(vm.count("starty")) m_FreetureCommandLineSettings.starty = vm["starty"].as<int>();

            // Cam width size
            if(vm.count("width")) m_FreetureCommandLineSettings.acqWidth = vm["width"].as<int>();

            // Cam height size
            if(vm.count("height")) m_FreetureCommandLineSettings.acqHeight = vm["height"].as<int>();

            // Gain value.
            if(vm.count("gain")) m_FreetureCommandLineSettings.gain = vm["gain"].as<int>();

            // Exposure value.
            if(vm.count("exposure")) m_FreetureCommandLineSettings.exp = vm["exposure"].as<double>();

            // Filename.
            if(vm.count("filename")) m_FreetureCommandLineSettings.fileName = vm["filename"].as<string>();

            if(vm.count("display")) m_FreetureCommandLineSettings.display = true;

            if(vm.count("bmp")) m_FreetureCommandLineSettings.bmp = true;

            if(vm.count("fits")) m_FreetureCommandLineSettings.fits = true;

            // Send fits by mail if configuration file is correct.
            if(vm.count("sendbymail")) m_FreetureCommandLineSettings.sendbymail = true;
    }

    if (m_CurrentRunMode == FreetureMode::UNKNOWN)
    {
        LOG_INFO << "MODE " << m_FreetureCommandLineSettings.mode << " is not available. Correct modes are : " << endl;
        LOG_INFO << "[1] Check configuration file." << endl;
        LOG_INFO << "[2] Run continous acquisition." << endl;
        LOG_INFO << "[3] Run meteor detection." << endl;
        LOG_INFO << "[4] Run single capture." << endl;
        LOG_INFO << "[5] Clean logs." << endl;
        LOG_INFO << "Execute freeture command to see options." << endl;
    }
}

/// <summary>
/// Print out version
/// </summary>
void Freeture::printVersion()
{
    LOG_INFO << "Current version : " << string(VERSION) << endl;
}

/// <summary>
/// Print help
/// </summary>
void Freeture::printHelp()
{
    LOG_INFO << *m_OptionsDescription << endl;
}

/// <summary>
/// Call Device list devices discovery
/// </summary>
void Freeture::selectListDevices()
{
    createDeviceManager();
    m_CameraDeviceManager->printDevicesList();
}

/// <summary>
/// Filter devices by pixel format
/// </summary>
void Freeture::selectListFormats()
{
    createDeviceManager();
    m_Device->getSupportedPixelFormats();
}

/// <summary>
/// Fetch program options
/// </summary>
void Freeture::fetchProgramOption()
{
    // Program options.
    m_OptionsDescription = new po::options_description("FreeTure options");

    m_OptionsDescription->add_options()
      ("help,h",                                                                                        "Print FreeTure help.")
      ("mode,m",        po::value<int>(),                                                               "FreeTure modes :\n- MODE 1 : Check configuration file.\n- MODE 2 : Continuous acquisition.\n- MODE 3 : Meteor detection.\n- MODE 4 : Single acquisition.\n- MODE 5 : Clean logs.")
      ("time,t",        po::value<int>(),                                                               "Execution time (s) of meteor detection mode.")
      ("startx",        po::value<int>(),                                                               "Crop starting x (from left to right): only for aravis cameras yet.")
      ("starty",        po::value<int>(),                                                               "Crop starting y (from top to bottom): only for aravis cameras yet.")
      ("width",         po::value<int>(),                                                               "Image width.")
      ("height",        po::value<int>(),                                                               "Image height.")
      ("cfg,c",         po::value<string>()->default_value(string(CFG_PATH) + "configuration.cfg"),     "Configuration file's path.")
      ("format,f",      po::value<int>()->default_value(0),                                             "Index of pixel format.")
      ("bmp",                                                                                           "Save .bmp.")
      ("fits",                                                                                          "Save fits2D.")
      ("gain,g",        po::value<int>(),                                                               "Define gain.")
      ("exposure,e",    po::value<double>(),                                                            "Define exposure.")
      ("version,v",                                                                                     "Print FreeTure version.")
      ("display",                                                                                       "Display the grabbed frame.")
      ("listdevices,l",                                                                                 "List connected devices.")
      ("listformats",                                                                                   "List device's available pixel formats.")
      ("id,d",          po::value<int>(),                                                               "Camera to use. List devices to get IDs.")
      ("filename,n",    po::value<string>()->default_value("snap"),                                     "Name to use when a single frame is captured.")
      ("sendbymail,s",                                                                                  "Send single capture by mail. Require -c option.")
      ("savepath,p",    po::value<string>()->default_value("./"),                                       "Save path.");

    po::variables_map vm;

    try
    {
        po::store(po::parse_command_line(m_Argc, m_Argv, *m_OptionsDescription), vm);

        if(vm.count("version"))
        {
            m_CurrentRunMode = FreetureMode::PRINT_VERSION;
        }
        else if(vm.count("help"))
        {
            m_CurrentRunMode = FreetureMode::PRINT_HELP;
        }
        else if(vm.count("listdevices"))
        {
            m_CurrentRunMode = FreetureMode::LIST_DEVICES;
        }
        else if(vm.count("listformats"))
        {
            m_CurrentRunMode = FreetureMode::LIST_FORMATS;

            if(vm.count("id"))
                m_FreetureCommandLineSettings.devID = vm["id"].as<int>();
        }
        else if(vm.count("mode"))
        {
            m_FreetureCommandLineSettings.mode = vm["mode"].as<int>();

            if(vm.count("cfg")) 
                m_FreetureCommandLineSettings.configurationFilePath = vm["cfg"].as<string>();

            if(vm.count("time"))
                m_FreetureCommandLineSettings.executionTime = vm["time"].as<int>();

            selectMode(vm);
        }
        else
        {
            m_CurrentRunMode = FreetureMode::PRINT_HELP;
        }
    }
    catch(exception& e)
    {
        LOG_ERROR << "Freeture::fetchProgramOption; "<<"Exception : " << e.what() << endl;
        return;
    }

    po::notify(vm);
}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%% MODE 1 : TEST/CHECK CONFIGURATION FILE %%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// Test the configuration file operative mode
/// </summary>
void Freeture::modeTest()
{
    bool error = false;

    LOG_INFO << "================================================" << endl;
    LOG_INFO << "====== FREETURE - Test/Check configuration =====" << endl;
    LOG_INFO << "================================================" << endl;

    createDeviceManager();


    m_FreetureSettings->enableErrors = true;

    LOG_INFO << "Testing configuration parameters..." << endl;

    if (!m_FreetureSettings->allParamAreCorrect()) {
        error = true;
        LOG_ERROR << "KO" << endl;
    }
    else
        LOG_INFO << "OK" << endl;

    LOG_INFO << "SELECTING DEVICE, CAMERA_ID=" << m_FreetureSettings->getAllParam().DEVICE_ID << ", CAMERA_SERIAL=" << m_FreetureSettings->getAllParam().CAMERA_SERIAL << "..." << endl;

    if (!m_CameraDeviceManager->selectDevice(m_FreetureSettings->getAllParam()))
        throw runtime_error("Failed to select device.");
    else {
        LOG_INFO << "OK" << endl;

        LOG_INFO << "Check input device configuration..." << endl;
        if (!m_FreetureSettings->checkInputParam(m_CameraDeviceManager->getDevice()->getDeviceType()))
        {
            error = true;
            LOG_ERROR << "KO" << endl;
        }
    }

    if (!error)
        LOG_INFO << "OK" << endl;
}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%% MODE 2 : CONTINUOUS ACQUISITION %%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// Continue acquisition mode
/// </summary>
void Freeture::modeContinuousAcquisition()
{
    LOG_INFO << "================================================" << endl;
    LOG_INFO << "========== FREETURE - Continuous mode ==========" << endl;
    LOG_INFO << "================================================" << endl;

    createDeviceManager();


    EParser<CamPixFmt> fmt;
    string fstring = fmt.getStringEnum(static_cast<CamPixFmt>(m_FreetureCommandLineSettings.acqFormat));
    if (fstring == "")
        throw runtime_error(">> Pixel format specified not found.");

    LOG_INFO << "------------------------------------------------" << endl;
    LOG_INFO << "CAM ID    : " << m_FreetureCommandLineSettings.devID << endl;
    LOG_INFO << "FORMAT    : " << fstring << endl;
    LOG_INFO << "GAIN      : " << m_FreetureCommandLineSettings.gain << endl;
    LOG_INFO << "EXPOSURE  : " << m_FreetureCommandLineSettings.exp << endl;
    LOG_INFO << "------------------------------------------------" << endl;

    LOG_INFO << "SELECTING DEVICE, CAMERA_ID=" << m_FreetureSettings->getAllParam().DEVICE_ID << ", CAMERA_SERIAL=" << m_FreetureSettings->getAllParam().CAMERA_SERIAL << "..." << endl;
    if (!m_CameraDeviceManager->selectDevice(m_FreetureSettings->getAllParam()))
        throw runtime_error("Fail to select device.");
    else
        LOG_INFO << "OK" << endl;

    m_Device->setCameraSize(m_FreetureCommandLineSettings.startx, m_FreetureCommandLineSettings.starty, m_FreetureCommandLineSettings.acqWidth, m_FreetureCommandLineSettings.acqHeight);

    m_Device->setCameraExposureTime(m_FreetureCommandLineSettings.exp);
    m_Device->setCameraGain(m_FreetureCommandLineSettings.gain);
    m_Device->initializeCamera();

    m_Device->startCamera(EAcquisitionMode::CONTINUOUS);

    if (m_FreetureCommandLineSettings.display)
        cv::namedWindow("FreeTure (ESC to stop)", cv::WINDOW_NORMAL);


    char hitKey;
    int interruption = 0;

    while (!interruption)
    {
        shared_ptr<Frame> frame = make_shared<Frame>();

        double tacq = (double)cv::getTickCount();
        if (m_Device->runContinuousCapture(frame)) {
            tacq = (((double)cv::getTickCount() - tacq) / cv::getTickFrequency()) * 1000;
            LOG_INFO << " >> [ TIME ACQ ] : " << tacq << " ms" << endl;

            if (m_FreetureCommandLineSettings.display) {
                cv::imshow("FreeTure (ESC to stop)", *frame->Image.get());
                cv::waitKey(1);
            }
        }

        /// Stop freeture if escape is pressed.


        if (interruption != 0) {
            hitKey = fgetc(stdin);
            if (hitKey == 27) interruption = 1;
            else interruption = 0;
        }

    }

    m_Device->stopCamera();

}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%% MODE 3 : METEOR DETECTION %%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// </summary>
void Freeture::modeMeteorDetection()
{
    boost::asio::io_service io_service;

    // Construct a signal set registered for process termination.
    boost::asio::signal_set signals(io_service, SIGINT);
    signals.async_wait(boost::bind(&Freeture::handler, this, boost::placeholders::_1, boost::placeholders::_2));

    LOG_INFO << "================================================" << endl;
    LOG_INFO << "======= FREETURE - Meteor detection mode =======" << endl;
    LOG_INFO << "================================================" << endl;

    createDeviceManager();

    /// ------------------------------------------------------------------
    /// --------------------- LOAD FREETURE PARAMETERS -------------------
    /// ------------------------------------------------------------------

    m_FreetureSettings->enableErrors = true;
    LOG_INFO << "CHECKING FREETURE CONFIGURATOION..." << endl;

    if (!m_FreetureSettings->allParamAreCorrect())
        throw runtime_error("Configuration file is not correct. Fail to launch detection mode.");
    else
        LOG_INFO << "OK" << endl;

    LOG_INFO << "SELECTING DEVICE, CAMERA_ID=" << m_FreetureSettings->getAllParam().DEVICE_ID << ", CAMERA_SERIAL=" << m_FreetureSettings->getAllParam().CAMERA_SERIAL << "..." << endl;

    if (!m_CameraDeviceManager->selectDevice(m_FreetureSettings->getAllParam()))
        throw runtime_error("Failed to select device.");
    else
        LOG_INFO << "OK" << endl;

    /// ------------------------------------------------------------------
    /// ------------------------- SHARED RESSOURCES ----------------------
    /// ------------------------------------------------------------------

    // Circular buffer to store last n grabbed frames.
    boost::circular_buffer<shared_ptr<Frame>> frameBuffer(m_FreetureSettings->getDetParam().ACQ_BUFFER_SIZE * m_FreetureSettings->getCamParam().ACQ_FPS);
    boost::mutex frameBuffer_m;
    boost::condition_variable frameBuffer_c;

    bool signalDet = false;
    boost::mutex signalDet_m;
    boost::condition_variable signalDet_c;

    bool signalStack = false;
    boost::mutex signalStack_m;
    boost::condition_variable signalStack_c;

    boost::mutex cfg_m;
    /// ------------------------------------------------------------------
    /// --------------------------- CREATE THREAD ------------------------
    /// ------------------------------------------------------------------

    try {

        // Create detection thread.
        if (m_FreetureSettings->getDetParam().DET_ENABLED) {
            LOG_INFO << "CREATE DETECTION THREAD..." << endl;

            m_DetectionThread = std::make_shared<DetThread>(frameBuffer,
                &frameBuffer_m,
                &frameBuffer_c,
                &signalDet,
                &signalDet_m,
                &signalDet_c,
                m_FreetureSettings);

            if (!m_DetectionThread->startThread())
                throw runtime_error("Fail to start detection thread.");
            else
                LOG_INFO << "OK" << endl;
        }



        // Create stack thread.
        if (m_FreetureSettings->getStackParam().STACK_ENABLED) {
            LOG_INFO << "CREATE STACK THREAD...." << endl;

            m_StackThread = std::make_shared< StackThread>(&signalStack,
                &signalStack_m,
                &signalStack_c,
                frameBuffer,
                &frameBuffer_m,
                &frameBuffer_c,
                m_FreetureSettings
                );

            if (!m_StackThread->startThread())
                throw runtime_error("Fail to start stack thread.");
            else {
                LOG_INFO << "OK" << endl;
#ifdef LINUX
                LOG_INFO << "This is the process ID : " << (unsigned long)getpid() << endl;
#endif
            }
        }

        LOG_INFO << "CREATE ACQUISITION THREAD..." << endl;
        // Create acquisition thread.
        m_AcquisitionThread = std::make_shared<AcqThread>(frameBuffer,
            &frameBuffer_m,
            &frameBuffer_c,
            &signalStack,
            &signalStack_m,
            &signalStack_c,
            &signalDet,
            &signalDet_m,
            &signalDet_c,
            m_DetectionThread,
            m_StackThread,
            m_FreetureSettings
            );


        if (!m_AcquisitionThread->startThread()) {
            throw runtime_error("Fail to start acquisition thread.");
        }
        else
        {
            LOG_INFO << "OK" << endl;

#ifdef LINUX
            LOG_INFO << "This is the process ID : " << (unsigned long)getpid() << endl;
#endif

            int cptTime = 0;
            bool waitLogTime = true;
            char hitKey;
            int interruption = 0;

            /// ------------------------------------------------------------------
            /// ----------------------------- MAIN LOOP --------------------------
            /// ------------------------------------------------------------------
            LOG_INFO << "START MAIN LOOP... PRESS ESC TO ABORT" << endl;

            while (!m_SigTermFlag && !interruption)
            {
#ifdef WINDOWS
                Sleep(1);
#else
                sleep(1);
#endif

                if (interruption != 0)
                {
                    hitKey = fgetc(stdin);
                    if (hitKey == 27) interruption = 1;
                    else interruption = 0;
                }


                /// Monitors logs.
                //logSystem.monitorLog();

                /// Stop freeture according time execution option.
                if (m_FreetureCommandLineSettings.executionTime != 0) {

                    if (cptTime > m_FreetureCommandLineSettings.executionTime) {
                        break;
                    }

                    cptTime++;

                }

                /// Stop freeture if one of the thread is stopped.
                if (m_AcquisitionThread->getThreadStatus()) {
                    break;
                }

                if (m_DetectionThread != nullptr) {
                    if (!m_DetectionThread->getRunStatus()) {
                        LOG_ERROR << "DetThread not running. Stopping the process ..." << endl;
                        break;
                    }
                }

                if (m_StackThread != nullptr) {
                    if (!m_StackThread->getRunStatus()) {
                        LOG_ERROR << "StackThread not running. Stopping the process ..." << endl;
                        break;
                    }
                }
            }
        }
    }
    catch (exception& e) {

        LOG_ERROR << e.what() << endl;

    }
    catch (const char* msg) {

        LOG_ERROR << msg << endl;

    }

    if (m_AcquisitionThread != nullptr) {
        m_AcquisitionThread->stopThread();
    }

    if (m_DetectionThread != nullptr) {
        m_DetectionThread->stopThread();
    }

    if (m_StackThread != nullptr) {
        m_StackThread->stopThread();
    }

}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%% MODE 4 : SINGLE ACQUISITION %%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// Run freeture as single acquisition
/// </summary>
void Freeture::modeSingleAcquisition()
{
    LOG_INFO << "================================================" << endl;
    LOG_INFO << "======== FREETURE - Single acquisition =========" << endl;
    LOG_INFO << "================================================" << endl;

    createDeviceManager();

    EParser<CamPixFmt> fmt;
    string fstring = fmt.getStringEnum(static_cast<CamPixFmt>(m_FreetureCommandLineSettings.acqFormat));
    if (fstring == "")
        throw runtime_error(">> Pixel format specified not found.");

    LOG_INFO << "------------------------------------------------" << endl;
    LOG_INFO << "CAM ID    : " << m_FreetureCommandLineSettings.devID << endl;
    LOG_INFO << "FORMAT    : " << fstring << endl;
    LOG_INFO << "GAIN      : " << m_FreetureCommandLineSettings.gain << endl;
    LOG_INFO << "EXPOSURE  : " << m_FreetureCommandLineSettings.exp << endl;

    if (m_FreetureCommandLineSettings.acqWidth > 0 && m_FreetureCommandLineSettings.acqHeight > 0) LOG_INFO << "SIZE      : " << m_FreetureCommandLineSettings.acqWidth << "x" << m_FreetureCommandLineSettings.acqHeight;
    if (m_FreetureCommandLineSettings.startx > 0 || m_FreetureCommandLineSettings.starty > 0) LOG_INFO << "START X,Y : " << m_FreetureCommandLineSettings.startx << "," << m_FreetureCommandLineSettings.starty;

    LOG_INFO << "SAVE PATH : " << m_FreetureCommandLineSettings.savePath << endl;
    LOG_INFO << "FILENAME  : " << m_FreetureCommandLineSettings.fileName << endl;
    LOG_INFO << "------------------------------------------------" << endl;

    /// ------------------------------------------------------------------
    /// ----------------------- MANAGE FILE NAME -------------------------
    /// ------------------------------------------------------------------

    int filenum = 0;
    bool increment = false;
    fs::path p(m_FreetureCommandLineSettings.savePath);

    // Search previous captures in the directory.
    for (directory_iterator file(p); file != directory_iterator(); ++file)
    {

        fs::path curr(file->path());

        if (is_regular_file(curr)) {

            list<string> ch;
            string fname = curr.filename().string();
            Conversion::stringTok(ch, fname.c_str(), "-");

            int i = 0;
            int n = 0;

            if (ch.front() == m_FreetureCommandLineSettings.fileName && ch.size() == 2) {

                list<string> ch_;
                Conversion::stringTok(ch_, ch.back().c_str(), ".");

                int nn = atoi(ch_.front().c_str());

                if (nn >= filenum) {

                    filenum = nn;
                    increment = true;

                }
            }
        }
    }

    if (increment) filenum++;

    /// ------------------------------------------------------------------
    /// ---------------------- RUN SINGLE CAPTURE ------------------------
    /// ------------------------------------------------------------------

    shared_ptr<Frame> frame = make_shared<Frame>();

    frame->mExposure = m_FreetureCommandLineSettings.exp;
    frame->mGain = m_FreetureCommandLineSettings.gain;
    frame->mFormat = static_cast<CamPixFmt>(m_FreetureCommandLineSettings.acqFormat);
    frame->mStartX = m_FreetureCommandLineSettings.startx != 0 ? m_FreetureCommandLineSettings.startx : m_FreetureSettings->getCamParam().ACQ_STARTX;
    frame->mStartY = m_FreetureCommandLineSettings.starty != 0 ? m_FreetureCommandLineSettings.starty : m_FreetureSettings->getCamParam().ACQ_STARTY;
    frame->mHeight = m_FreetureCommandLineSettings.acqHeight != 0 ? m_FreetureCommandLineSettings.acqHeight : m_FreetureSettings->getCamParam().ACQ_HEIGHT;
    frame->mWidth = m_FreetureCommandLineSettings.acqWidth != 0 ? m_FreetureCommandLineSettings.acqWidth : m_FreetureSettings->getCamParam().ACQ_WIDTH;

    LOG_INFO << "++++++++++++++++++++++++++ startx " << frame->mStartX << endl;
    LOG_INFO << "++++++++++++++++++++++++++ starty " << frame->mStartY << endl;
    LOG_INFO << "++++++++++++++++++++++++++ acqHeight " << frame->mHeight << endl;
    LOG_INFO << "++++++++++++++++++++++++++ acqWidth " << frame->mWidth << endl;

    m_Device->setCameraGain(m_FreetureCommandLineSettings.gain);
    m_Device->setCameraExposureTime(m_FreetureCommandLineSettings.exp);

    if (!m_Device->runSingleCapture(frame)) {
        throw runtime_error(">> Single capture failed.");
    }


    LOG_INFO << ">> Single capture succeed." << endl;

    /// ------------------------------------------------------------------
    /// ------------------- SAVE / DISPLAY CAPTURE -----------------------
    /// ------------------------------------------------------------------

    if (frame->Image->data) {

        // Save the frame in BMP.
        if (m_FreetureCommandLineSettings.bmp) {

            LOG_INFO << ">> Saving bmp file ..." << endl;

            cv::Mat temp1, newMat;
            frame->Image->copyTo(temp1);

            if (frame->mFormat == CamPixFmt::MONO12) {
                newMat = ImgProcessing::correctGammaOnMono12(temp1, 2.2);
                cv::Mat temp = Conversion::convertTo8UC1(newMat);
            }
            else {
                newMat = ImgProcessing::correctGammaOnMono8(temp1, 2.2);
            }

            SaveImg::saveBMP(newMat, m_FreetureCommandLineSettings.savePath + m_FreetureCommandLineSettings.fileName + "-" + Conversion::intToString(filenum));
            LOG_INFO << ">> Bmp saved : " << m_FreetureCommandLineSettings.savePath << m_FreetureCommandLineSettings.fileName << "-" << Conversion::intToString(filenum) << ".bmp" << endl;

        }

        // Save the frame in Fits 2D.
        if (m_FreetureCommandLineSettings.fits) {

            LOG_INFO << ">> Saving fits file ..." << endl;

            Fits fh;
            bool useCfg = false;
            Fits2D newFits(m_FreetureCommandLineSettings.savePath);

            m_FreetureSettings->enableErrors = true;
            if (m_FreetureSettings->checkStationParam() && m_FreetureSettings->checkFitskeysParam()) {
                useCfg = true;
                double  debObsInSeconds = frame->mDate.hours * 3600 + frame->mDate.minutes * 60 + frame->mDate.seconds;
                double  julianDate = TimeDate::gregorianToJulian(frame->mDate);
                double  julianCentury = TimeDate::julianCentury(julianDate);
                double  sideralT = TimeDate::localSideralTime_2(julianCentury, frame->mDate.hours, frame->mDate.minutes, (int)frame->mDate.seconds, fh.kSITELONG);
                newFits.kCRVAL1 = sideralT;
                newFits.loadKeys(m_FreetureSettings->getFitskeysParam(), m_FreetureSettings->getStationParam());

            }

            newFits.kGAINDB = (int)m_FreetureCommandLineSettings.gain;
            newFits.kELAPTIME = m_FreetureCommandLineSettings.exp / 1000000.0;
            newFits.kEXPOSURE = m_FreetureCommandLineSettings.exp / 1000000.0;
            newFits.kONTIME = m_FreetureCommandLineSettings.exp / 1000000.0;
            newFits.kDATEOBS = TimeDate::getIsoExtendedFormatDate(frame->mDate);
            newFits.kCTYPE1 = "RA---ARC";
            newFits.kCTYPE2 = "DEC--ARC";
            newFits.kEQUINOX = 2000.0;

            if (frame->mFormat == CamPixFmt::MONO12) {
                // Create FITS image with BITPIX = SHORT_IMG (16-bits signed integers), pixel with TSHORT (signed short)
                if (newFits.writeFits(frame->Image, S16, m_FreetureCommandLineSettings.fileName + "-" + Conversion::intToString(filenum)))
                    LOG_INFO << ">> Fits saved in : " << m_FreetureCommandLineSettings.savePath << m_FreetureCommandLineSettings.fileName << "-" << Conversion::intToString(filenum) << ".fit" << endl;
            }
            else {
                // Create FITS image with BITPIX = BYTE_IMG (8-bits unsigned integers), pixel with TBYTE (8-bit unsigned byte)
                if (newFits.writeFits(frame->Image, UC8, m_FreetureCommandLineSettings.fileName + "-" + Conversion::intToString(filenum)))
                    LOG_INFO << ">> Fits saved in : " << m_FreetureCommandLineSettings.savePath << m_FreetureCommandLineSettings.fileName << "-" << Conversion::intToString(filenum) << ".fit" << endl;

            }

            // Send fits by mail if configuration file is correct.
            if (m_FreetureCommandLineSettings.sendbymail && useCfg) {

                if (m_FreetureSettings->checkMailParam()) {

                    vector<string> mailAttachments;
                    mailAttachments.push_back(m_FreetureCommandLineSettings.savePath + m_FreetureCommandLineSettings.fileName + "-" + Conversion::intToString(filenum) + ".fit");

                    SMTPClient::sendMail(m_FreetureSettings->getMailParam().MAIL_SMTP_SERVER,
                        m_FreetureSettings->getMailParam().MAIL_SMTP_LOGIN,
                        m_FreetureSettings->getMailParam().MAIL_SMTP_PASSWORD,
                        "freeture@snap",
                        m_FreetureSettings->getMailParam().MAIL_RECIPIENTS,
                        m_FreetureCommandLineSettings.fileName + "-" + Conversion::intToString(filenum) + ".fit",
                        " Exposure time : " + Conversion::intToString((int)m_FreetureCommandLineSettings.exp) + "\n Gain : " + Conversion::intToString((int)m_FreetureCommandLineSettings.gain),
                        mailAttachments,
                        m_FreetureSettings->getMailParam().MAIL_CONNECTION_TYPE);

                }
            }

        }

        // Display the frame in an opencv window
        if (m_FreetureCommandLineSettings.display) {

            LOG_INFO << ">> Display single capture." << endl;

            cv::Mat temp, temp1;
            frame->Image->copyTo(temp1);

            if (frame->mFormat == CamPixFmt::MONO12) {
                cv::Mat gammaCorrected = ImgProcessing::correctGammaOnMono12(temp1, 2.2);
                temp = Conversion::convertTo8UC1(gammaCorrected);
            }
            else {
                temp = ImgProcessing::correctGammaOnMono8(temp1, 2.2);
            }

            cv::namedWindow("FreeTure (Press a key to close)", cv::WINDOW_NORMAL);
            cv::imshow("FreeTure (Press a key to close)", temp);
            cv::waitKey(0);

        }
    }

}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%% MODE 5 : CLEAN LOGS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// Run freeture as Clean logs
/// </summary>
void Freeture::modeCleanLogs()
{
    LOG_INFO << "================================================" << endl;
    LOG_INFO << "======== FREETURE - Clean logs         =========" << endl;
    LOG_INFO << "================================================" << endl;

    // Simply remove all log directory contents.
    boost::filesystem::path p(m_FreetureSettings->getLogParam().LOG_PATH);

    if(boost::filesystem::exists(p)){
        boost::filesystem::remove_all(p);
        LOG_INFO << "Clean log completed." << endl;
    }else {
        LOG_INFO << "Log directory not found." << endl;
    }
}



/// <summary>
/// Main freeture routine function, routes program options to the correct operative function
/// </summary>
void Freeture::Run()
{
    try {
        m_ThreadID = std::this_thread::get_id();

        fetchProgramOption();

        m_Logger = Logger::Get(LogThread::FREETURE_THREAD, m_ThreadID, m_FreetureSettings->getLogParam().LOG_PATH, m_FreetureSettings->getLogParam().LOG_ARCHIVE_DAY, m_FreetureSettings->getLogParam().LOG_SIZE_LIMIT, m_FreetureSettings->getLogParam().LOG_SEVERITY);
        m_Logger->setLogThread(LogThread::FREETURE_THREAD, m_ThreadID);

        LOG_INFO << "================================================" << endl;
        LOG_INFO << "======        FREETURE - " << VERSION << "           ======= " << endl;
        LOG_INFO << "================================================" << endl;


        EParser<FreetureMode> mode_parser;
        string mode_string = mode_parser.getStringEnum(m_CurrentRunMode);
        LOG_INFO << "FREETURE RUNNING MODE:\t" << mode_string << endl;

        EParser<LogSeverityLevel> sev_parser;
        string sev_string = sev_parser.getStringEnum(m_FreetureSettings->getLogParam().LOG_SEVERITY);
        LOG_INFO << "LOGGING MODE\t\t: " << sev_string << endl;
        LOG_INFO << "LOG PATH\t\t\t: " << m_FreetureSettings->getLogParam().LOG_PATH << endl;
        LOG_INFO << "LOG ARCHIVE\t\t: " << m_FreetureSettings->getLogParam().LOG_ARCHIVE_DAY<<"[days]" << endl;
        LOG_INFO << "LOG SIZE LIMIT\t\t: " << m_FreetureSettings->getLogParam().LOG_SIZE_LIMIT << "[MB]" << endl;



        switch (m_CurrentRunMode)
        {
        case FreetureMode::PRINT_VERSION:
        {
            printVersion();
            break;
        }
        case FreetureMode::LIST_FORMATS:
        {
            selectListFormats();
            break;
        }
        case FreetureMode::LIST_DEVICES:
        {
            selectListDevices();
            break;
        }
        case FreetureMode::PRINT_HELP:
        {
            printHelp();
            break;
        }
        case FreetureMode::CONTINUOUS_ACQUISITION:
        {
            modeContinuousAcquisition();
            break;
        }
        case FreetureMode::TEST_CONFIGURATION:
        {
            modeTest();
            break;
        }
        case FreetureMode::METEOR_DETECTION:
        {
            modeMeteorDetection();
            break;
        }
        case FreetureMode::SINGLE_ACQUISITION:
        {
            modeSingleAcquisition();
            break;
        }
        case FreetureMode::CLEAN_LOGS:
        {
            modeCleanLogs();
            break;
        }
        }
    }
    catch (exception& e) {
        LOG_ERROR << e.what() << endl;
    }
    catch (...) {
        LOG_ERROR << "An error occured." << endl;
    }
}

void Freeture::createDeviceManager()
{
    //load device manager
    switch (m_CurrentRunMode)
    {
    case FreetureMode::LIST_FORMATS:
    case FreetureMode::LIST_DEVICES:
    case FreetureMode::CONTINUOUS_ACQUISITION:
    case FreetureMode::TEST_CONFIGURATION:
    case FreetureMode::METEOR_DETECTION:
    case FreetureMode::SINGLE_ACQUISITION:

        LOG_INFO << "SCANNING DEVICES..." << endl;
        m_CameraDeviceManager = CameraDeviceManager::Get();

        size_t device_count = m_CameraDeviceManager->getDeviceCount();
        
        if (device_count == 0) {
            throw runtime_error("NO DEVICES AVAIALABLE");
        }

        if (device_count != 1)
            LOG_INFO << "FOUND " << device_count << " DEVICES!" << endl;
        else
            LOG_INFO << "FOUND " << device_count << " DEVICE!" << endl;

        m_Device = m_CameraDeviceManager->getDevice();
        return;
    }

    throw runtime_error("This running mode have not camera device manager feature");
}
