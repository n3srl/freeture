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
void freeture::Freeture::signalHandler(int signum)
{
    LOG_WARNING << "Freeture::signalHandler;" << "Received signal : " << signum;
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
void freeture::Freeture::selectMode( boost::program_options::variables_map& vm)
{
    m_CurrentRunMode = Mode::UNKNOWN;

    switch(m_FreetureCommandLineSettings.mode)
    {
        case 1:
        {
            m_CurrentRunMode = Mode::TEST_CONFIGURATION;
        }
        break;
        case 2 :
        {
            m_CurrentRunMode = Mode::CONTINUOUS_ACQUISITION;
        }
        break;
        case 3:
        {
            m_CurrentRunMode = Mode::METEOR_DETECTION;
        }
        break;
        case 4 :
        {
            m_CurrentRunMode = Mode::SINGLE_ACQUISITION;
        }
        break;
        case 5:
        {
            m_CurrentRunMode = Mode::CLEAN_LOGS;
        }
        break;
    }

    if (
        m_CurrentRunMode == Mode::METEOR_DETECTION ||
        m_CurrentRunMode == Mode::SINGLE_ACQUISITION ||
        m_CurrentRunMode == Mode::CONTINUOUS_ACQUISITION
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

    if (m_CurrentRunMode == freeture::Mode::UNKNOWN)
    {
        LOG_INFO << "MODE " << m_FreetureCommandLineSettings.mode << " is not available. Correct modes are : " <<endl;
        LOG_INFO << "[1] Check configuration file."                         ;
        LOG_INFO << "[2] Run continous acquisition."                        ;
        LOG_INFO << "[3] Run meteor detection."                             ;
        LOG_INFO << "[4] Run single capture."                               ;
        LOG_INFO << "[5] Clean logs."                                        << endl;

        LOG_INFO << "Execute freeture command to see options." ;
    }
}

/// <summary>
/// Print out version
/// </summary>
void freeture::Freeture::printVersion()
{
    LOG_INFO << "Current version : " << string(VERSION) ;
}

/// <summary>
/// Print help
/// </summary>
void freeture::Freeture::printHelp()
{
    LOG_INFO << *desc;
}

/// <summary>
/// Call Device list devices discovery
/// </summary>
void freeture::Freeture::selectListDevices()
{
   Device* device = m_CameraDeviceManager->getDevice();

   device = new Device();
   device->listDevices(true);
   delete device;
}

/// <summary>
/// Filter devices by pixel format
/// </summary>
void freeture::Freeture::selectListFormats()
{
    Device* device = m_CameraDeviceManager->getDevice();

    device->listDevices(false);

    if(device->createCamera(m_FreetureCommandLineSettings.devID, true))
    {
        device->getSupportedPixelFormats();
    }

    delete device;
}

/// <summary>
/// Fetch program options
/// </summary>
void freeture::Freeture::fetchProgramOption()
{
    // Program options.
    desc = new po::options_description("FreeTure options");

    desc->add_options()
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
        po::store(po::parse_command_line(m_Argc, m_Argv, *desc), vm);

        if(vm.count("version"))
        {
            m_CurrentRunMode = freeture::Mode::PRINT_VERSION;
        }
        else if(vm.count("help"))
        {
            m_CurrentRunMode = freeture::Mode::PRINT_HELP;
        }
        else if(vm.count("listdevices"))
        {
            m_CurrentRunMode = freeture::Mode::LIST_DEVICES;
        }
        else if(vm.count("listformats"))
        {
            m_CurrentRunMode = freeture::Mode::LIST_FORMATS;

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
            m_CurrentRunMode = freeture::Mode::PRINT_HELP;
        }
    }
    catch(exception& e)
    {
        LOG_ERROR << "Freeture::fetchProgramOption; "<<"Exception : " << e.what() ;
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
void freeture::Freeture::modeTest()
{
    bool error = false;
    LOG_INFO << "================================================";
    LOG_INFO << "====== FREETURE - Test/Check configuration =====";
    LOG_INFO << "================================================" << endl;

    m_FreetureSettings->enableErrors = true;
    
    if (!m_FreetureSettings->allParamAreCorrect()) {
        error = true;
        LOG_ERROR << "CONFIGURATION ERROR";
    }
    
    m_CameraDeviceManager = CameraDeviceManager::Get();

    m_CameraDeviceManager->selectDevice(m_FreetureSettings->getAllParam().DEVICE_ID);

    if (!m_FreetureSettings->checkInputParam(m_CameraDeviceManager->getDevice()->getDeviceType())) 
    {
        error = true;
        LOG_ERROR << "INPUT DEVICE CONFIGURATION ERROR";
    }

    if (!error)
        LOG_INFO << "CONFIGURATION OK";
}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%% MODE 2 : CONTINUOUS ACQUISITION %%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// Continue acquisition mode
/// </summary>
void freeture::Freeture::modeContinuousAcquisition()
{
    LOG_INFO << "================================================" ;
    LOG_INFO << "========== FREETURE - Continuous mode ==========" ;
    LOG_INFO << "================================================"  << endl;

    EParser<CamPixFmt> fmt;
    string fstring = fmt.getStringEnum(static_cast<CamPixFmt>(m_FreetureCommandLineSettings.acqFormat));
    if(fstring == "")
        throw ">> Pixel format specified not found.";

    LOG_INFO << "------------------------------------------------" ;
    LOG_INFO << "CAM ID    : " << m_FreetureCommandLineSettings.devID ;
    LOG_INFO << "FORMAT    : " << fstring ;
    LOG_INFO << "GAIN      : " << m_FreetureCommandLineSettings.gain ;
    LOG_INFO << "EXPOSURE  : " << m_FreetureCommandLineSettings.exp ;
    LOG_INFO << "------------------------------------------------"  << endl;

    Device* device = m_CameraDeviceManager->getDevice();

    device->listDevices(false);
    device->mFormat = static_cast<CamPixFmt>(m_FreetureCommandLineSettings.acqFormat);

    if(!device->createCamera(m_FreetureCommandLineSettings.devID, true)) {
        throw "Fail to create device.";
    }

    if(m_FreetureCommandLineSettings.acqWidth != 0 && m_FreetureCommandLineSettings.acqHeight != 0)
        device->setCameraSize(m_FreetureCommandLineSettings.startx, m_FreetureCommandLineSettings.starty, m_FreetureCommandLineSettings.acqWidth, m_FreetureCommandLineSettings.acqHeight);
    else
        device->setCameraSize();

    if(!device->setCameraPixelFormat()) {
        throw "Fail to set format";
    }

    device->setCameraFPS();
    device->setCameraExposureTime(m_FreetureCommandLineSettings.exp);
    device->setCameraGain(m_FreetureCommandLineSettings.gain);
    device->initializeCamera();
    device->startCamera();

    if(m_FreetureCommandLineSettings.display)
        cv::namedWindow("FreeTure (ESC to stop)", cv::WINDOW_NORMAL);


    char hitKey;
    int interruption = 0;

    while(!interruption)
    {
                            Frame frame;

                            double tacq = (double)cv::getTickCount();
                            if(device->runContinuousCapture(frame)){
                                tacq = (((double)cv::getTickCount() - tacq)/ cv::getTickFrequency())*1000;
                                LOG_INFO << " >> [ TIME ACQ ] : " << tacq << " ms" ;

                                if(m_FreetureCommandLineSettings.display) {
                                    cv::imshow("FreeTure (ESC to stop)", frame.mImg);
                                    cv::waitKey(1);
                                }
                            }

                            /// Stop freeture if escape is pressed.


                            if(interruption !=0) {
                                hitKey = fgetc(stdin);
                                if(hitKey == 27) interruption = 1;
                                else interruption = 0;
                            }

    }

    device->stopCamera();
    delete device;
}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%% MODE 3 : METEOR DETECTION %%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// </summary>
void freeture::Freeture::modeMeteorDetection()
{
    LOG_INFO << "================================================";
    LOG_INFO << "======= FREETURE - Meteor detection mode =======";
    LOG_INFO << "================================================" << endl;
    
   /// ------------------------------------------------------------------
   /// --------------------- LOAD FREETURE PARAMETERS -------------------
   /// ------------------------------------------------------------------

   m_FreetureSettings->enableErrors = true;
   if(!m_FreetureSettings->allParamAreCorrect())
        throw "Configuration file is not correct. Fail to launch detection mode.";
   
    Device* device = m_CameraDeviceManager->getDevice();

    device->Setup(m_FreetureSettings->getCamParam(), m_FreetureSettings->getFramesParam(), m_FreetureSettings->getVidParam(), m_FreetureCommandLineSettings.devID);


    //device->Setup(cfg.getCamParam(), cfg.getFramesParam(), cfg.getVidParam(), devID);

    //Logger logSystem(cfg.getLogParam().LOG_PATH, cfg.getLogParam().LOG_ARCHIVE_DAY, cfg.getLogParam().LOG_SIZE_LIMIT, logFiles);
    
    /// ------------------------------------------------------------------
    /// -------------------------- MANAGE LOG ----------------------------
    /// ------------------------------------------------------------------
    fs::path pLog(m_FreetureSettings->getLogParam().LOG_PATH);
    if(!fs::exists(pLog))
    {

                            if(!fs::create_directory(pLog))
                                throw "> Failed to create a directory for logs files.";
                            else
                                LOG_INFO << "> Log directory created : " << pLog ;
    }

    
    LOG_INFO << "\n";
    LOG_INFO << "==============================================";
    LOG_INFO << "====== FREETURE- Meteor detection mode ======";
    LOG_INFO << "==============================================";

    /// ------------------------------------------------------------------
    /// ------------------------- SHARED RESSOURCES ----------------------
    /// ------------------------------------------------------------------
   
    // Circular buffer to store last n grabbed frames.
    boost::circular_buffer<Frame> frameBuffer(m_FreetureSettings->getDetParam().ACQ_BUFFER_SIZE * m_FreetureSettings->getCamParam().ACQ_FPS);
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
                            if(m_FreetureSettings->getDetParam().DET_ENABLED) {

                                LOG_INFO << "Start to create detection thread.";
                                LOG_INFO << "Start to create detection thread." ;

                                m_DetectionThread = std::make_shared<DetThread> ( &frameBuffer,
                                                            &frameBuffer_m,
                                                            &frameBuffer_c,
                                                            &signalDet,
                                                            &signalDet_m,
                                                            &signalDet_c,
                                                            m_FreetureSettings);

                                if(!m_DetectionThread->startThread())
                                    throw "Fail to start detection thread.";

                            }

                            

                            // Create stack thread.
                            if(m_FreetureSettings->getStackParam().STACK_ENABLED) {

                                LOG_INFO << "Start to create stack Thread.";

                                m_StackThread = std::make_shared< StackThread>( &signalStack,
                                                                &signalStack_m,
                                                                &signalStack_c,
                                                                &frameBuffer,
                                                                &frameBuffer_m,
                                                                &frameBuffer_c,
                                                                m_FreetureSettings
                                    );

                                if(!m_StackThread->startThread())
                                    throw "Fail to start stack thread.";

                            }

                            
                            
                            // Create acquisition thread.
                            m_AcquisitionThread = std::make_shared<AcqThread>( &frameBuffer,
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

                            if(!m_AcquisitionThread->startThread()) {

                                throw "Fail to start acquisition thread.";

                            }else {

                                

                                LOG_INFO << "Success to start acquisition Thread.";

                                #ifdef LINUX
                                LOG_INFO << "This is the process : " << (unsigned long)getpid();
                                #endif

                                int cptTime = 0;
                                bool waitLogTime = true;
                                char hitKey;
                                int interruption = 0;

                                /// ------------------------------------------------------------------
                                /// ----------------------------- MAIN LOOP --------------------------
                                /// ------------------------------------------------------------------

                                while(!m_SigTermFlag && !interruption) {


#ifdef WINDOWS
                                    Sleep(1);
#else
                                    sleep(1);
#endif

                                    if(interruption !=0) {
                                        hitKey = fgetc(stdin);
                                        if(hitKey == 27) interruption = 1;
                                        else interruption = 0;
                                    }


                                    /// Monitors logs.
                                    //logSystem.monitorLog();

                                    /// Stop freeture according time execution option.
                                    if(m_FreetureCommandLineSettings.executionTime != 0) {

                                        if(cptTime > m_FreetureCommandLineSettings.executionTime){
                                            break;
                                        }

                                        cptTime ++;

                                    }

                                    /// Stop freeture if one of the thread is stopped.
                                    if(m_AcquisitionThread->getThreadStatus()){
                                        break;
                                    }

                                    if(m_DetectionThread != NULL){
                                        if(!m_DetectionThread->getRunStatus()){
                                            LOG_ERROR << "DetThread not running. Stopping the process ...";
                                            break;
                                        }
                                    }

                                    if(m_StackThread != NULL){
                                        if(!m_StackThread->getRunStatus()){
                                            LOG_ERROR << "StackThread not running. Stopping the process ...";
                                            break;
                                        }
                                    }
                                }

                            }

                        }catch(exception& e) {
                            
                            LOG_ERROR << e.what();

                        }catch(const char * msg) {

                            LOG_ERROR << msg;

                        }

                        if(m_AcquisitionThread != NULL) {
                            m_AcquisitionThread->stopThread();
                        }

                        if(m_DetectionThread != NULL) {
                            m_DetectionThread->stopThread();
                        }

                        if(m_StackThread != NULL){
                            m_StackThread->stopThread();
                        }
}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%% MODE 4 : SINGLE ACQUISITION %%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// Run freeture as single acquisition
/// </summary>
void freeture::Freeture::modeSingleAcquisition()
{
    LOG_INFO << "================================================";
    LOG_INFO << "======== FREETURE - Single acquisition =========";
    LOG_INFO << "================================================" << endl;
    
    Device* device = m_CameraDeviceManager->getDevice();

    try
    {
        device->Setup(m_FreetureSettings->getCamParam(), m_FreetureSettings->getFramesParam(), m_FreetureSettings->getVidParam(), m_FreetureCommandLineSettings.devID);

        
        EParser<CamPixFmt> fmt;
        string fstring = fmt.getStringEnum(static_cast<CamPixFmt>(m_FreetureCommandLineSettings.acqFormat));
        if(fstring == "")
            throw ">> Pixel format specified not found.";

        LOG_INFO << "------------------------------------------------" ;
        LOG_INFO << "CAM ID    : " << m_FreetureCommandLineSettings.devID ;
        LOG_INFO << "FORMAT    : " << fstring ;
        LOG_INFO << "GAIN      : " << m_FreetureCommandLineSettings.gain ;
        LOG_INFO << "EXPOSURE  : " << m_FreetureCommandLineSettings.exp ;

        if(m_FreetureCommandLineSettings.acqWidth > 0 && m_FreetureCommandLineSettings.acqHeight > 0) LOG_INFO << "SIZE      : " << m_FreetureCommandLineSettings.acqWidth << "x" << m_FreetureCommandLineSettings.acqHeight ;
        if(m_FreetureCommandLineSettings.startx > 0 || m_FreetureCommandLineSettings.starty > 0) LOG_INFO << "START X,Y : " << m_FreetureCommandLineSettings.startx << "," << m_FreetureCommandLineSettings.starty ;

        LOG_INFO << "SAVE PATH : " << m_FreetureCommandLineSettings.savePath ;
        LOG_INFO << "FILENAME  : " << m_FreetureCommandLineSettings.fileName ;
        LOG_INFO << "------------------------------------------------"  << endl;

        /// ------------------------------------------------------------------
        /// ----------------------- MANAGE FILE NAME -------------------------
        /// ------------------------------------------------------------------

        int filenum = 0;
        bool increment = false;
        fs::path p(m_FreetureCommandLineSettings.savePath);

        // Search previous captures in the directory.
        for(directory_iterator file(p);file!= directory_iterator(); ++file)
        {

                            fs::path curr(file->path());

                            if(is_regular_file(curr)) {

                                list<string> ch;
                                string fname = curr.filename().string();
                                Conversion::stringTok(ch, fname.c_str(), "-");

                                int i = 0;
                                int n = 0;

                                if(ch.front() == m_FreetureCommandLineSettings.fileName && ch.size() == 2) {

                                    list<string> ch_;
                                    Conversion::stringTok(ch_, ch.back().c_str(), ".");

                                    int nn = atoi(ch_.front().c_str());

                                    if(nn >= filenum){

                                        filenum = nn;
                                        increment = true;

                                    }
                                }
                            }
    }

                        if(increment) filenum++;

                        /// ------------------------------------------------------------------
                        /// ---------------------- RUN SINGLE CAPTURE ------------------------
                        /// ------------------------------------------------------------------

                        int devStartx, devStarty, devHeight, devWidth = 0;

                        bool cameraParamsGet = device->getDeviceCameraSizeParams(devStartx, devStarty, devHeight, devWidth);
                        
                        Frame frame;
                        frame.mExposure = m_FreetureCommandLineSettings.exp;
                        frame.mGain = m_FreetureCommandLineSettings.gain;
                        frame.mFormat = static_cast<CamPixFmt>(m_FreetureCommandLineSettings.acqFormat);
                        frame.mStartX = m_FreetureCommandLineSettings.startx != 0 ? m_FreetureCommandLineSettings.startx : devStartx;
                        frame.mStartY = m_FreetureCommandLineSettings.starty != 0 ? m_FreetureCommandLineSettings.starty : devStarty;
                        frame.mHeight = m_FreetureCommandLineSettings.acqHeight != 0 ? m_FreetureCommandLineSettings.acqHeight : devHeight;
                        frame.mWidth = m_FreetureCommandLineSettings.acqWidth != 0 ? m_FreetureCommandLineSettings.acqWidth : devWidth;

                        LOG_INFO << "++++++++++++++++++++++++++ startx " << frame.mStartX ;
                        LOG_INFO << "++++++++++++++++++++++++++ starty " << frame.mStartY ;
                        LOG_INFO << "++++++++++++++++++++++++++ acqHeight " << frame.mHeight ;
                        LOG_INFO << "++++++++++++++++++++++++++ acqWidth " << frame.mWidth ;


                       
                        device->setCameraAutoExposure(false);
                        device->setCameraGain(m_FreetureCommandLineSettings.gain);
                        device->setCameraExposureTime(m_FreetureCommandLineSettings.exp);

                        

                        if(!device->runSingleCapture(frame)){
                            throw ">> Single capture failed.";
                        }


                        LOG_INFO << ">> Single capture succeed." ;

                        /// ------------------------------------------------------------------
                        /// ------------------- SAVE / DISPLAY CAPTURE -----------------------
                        /// ------------------------------------------------------------------

                        if(frame.mImg.data) {

                            // Save the frame in BMP.
                            if(m_FreetureCommandLineSettings.bmp) {

                                LOG_INFO << ">> Saving bmp file ..." ;

                                cv::Mat temp1, newMat;
                                frame.mImg.copyTo(temp1);

                                if(frame.mFormat == MONO12){
                                    newMat = ImgProcessing::correctGammaOnMono12(temp1, 2.2);
                                    cv::Mat temp = Conversion::convertTo8UC1(newMat);
                                }else {
                                    newMat = ImgProcessing::correctGammaOnMono8(temp1, 2.2);
                                }

                                SaveImg::saveBMP(newMat, m_FreetureCommandLineSettings.savePath + m_FreetureCommandLineSettings.fileName + "-" + Conversion::intToString(filenum));
                                LOG_INFO << ">> Bmp saved : " << m_FreetureCommandLineSettings.savePath << m_FreetureCommandLineSettings.fileName << "-" << Conversion::intToString(filenum) << ".bmp" ;

                            }

                            // Save the frame in Fits 2D.
                            if(m_FreetureCommandLineSettings.fits) {

                                LOG_INFO << ">> Saving fits file ..." ;

                                Fits fh;
                                bool useCfg = false;
                                Fits2D newFits(m_FreetureCommandLineSettings.savePath);

                                m_FreetureSettings->enableErrors = true;
                                if(m_FreetureSettings->checkStationParam() && m_FreetureSettings->checkFitskeysParam()) {
                                    useCfg = true;
                                    double  debObsInSeconds = frame.mDate.hours*3600 + frame.mDate.minutes*60 + frame.mDate.seconds;
                                    double  julianDate      = TimeDate::gregorianToJulian(frame.mDate);
                                    double  julianCentury   = TimeDate::julianCentury(julianDate);
                                    double  sideralT        = TimeDate::localSideralTime_2(julianCentury, frame.mDate.hours, frame.mDate.minutes, (int)frame.mDate.seconds, fh.kSITELONG);
                                    newFits.kCRVAL1 = sideralT;
                                    newFits.loadKeys(m_FreetureSettings->getFitskeysParam(), m_FreetureSettings->getStationParam());

                                }

                                newFits.kGAINDB = (int)m_FreetureCommandLineSettings.gain;
                                newFits.kELAPTIME = m_FreetureCommandLineSettings.exp/1000000.0;
                                newFits.kEXPOSURE = m_FreetureCommandLineSettings.exp/1000000.0;
                                newFits.kONTIME = m_FreetureCommandLineSettings.exp/1000000.0;
                                newFits.kDATEOBS = TimeDate::getIsoExtendedFormatDate(frame.mDate);
                                newFits.kCTYPE1 = "RA---ARC";
                                newFits.kCTYPE2 = "DEC--ARC";
                                newFits.kEQUINOX = 2000.0;

                                if(frame.mFormat == MONO12){
                                    // Create FITS image with BITPIX = SHORT_IMG (16-bits signed integers), pixel with TSHORT (signed short)
                                    if(newFits.writeFits(frame.mImg, S16, m_FreetureCommandLineSettings.fileName + "-" + Conversion::intToString(filenum)))
                                        LOG_INFO << ">> Fits saved in : " << m_FreetureCommandLineSettings.savePath << m_FreetureCommandLineSettings.fileName << "-" << Conversion::intToString(filenum) << ".fit" ;
                                }else{
                                    // Create FITS image with BITPIX = BYTE_IMG (8-bits unsigned integers), pixel with TBYTE (8-bit unsigned byte)
                                    if(newFits.writeFits(frame.mImg, UC8, m_FreetureCommandLineSettings.fileName + "-" + Conversion::intToString(filenum)))
                                        LOG_INFO << ">> Fits saved in : " << m_FreetureCommandLineSettings.savePath << m_FreetureCommandLineSettings.fileName << "-" << Conversion::intToString(filenum) << ".fit" ;

                                }

                                // Send fits by mail if configuration file is correct.
                                if(m_FreetureCommandLineSettings.sendbymail && useCfg) {

                                    if(m_FreetureSettings->checkMailParam()) {

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
                            if(m_FreetureCommandLineSettings.display) {

                                LOG_INFO << ">> Display single capture." ;

                                cv::Mat temp, temp1;
                                frame.mImg.copyTo(temp1);

                                if(frame.mFormat == MONO12) {
                                    cv::Mat gammaCorrected = ImgProcessing::correctGammaOnMono12(temp1,2.2);
                                    temp = Conversion::convertTo8UC1(gammaCorrected);
                                }else{
                                    temp = ImgProcessing::correctGammaOnMono8(temp1,2.2);
                                }

                                cv::namedWindow("FreeTure (Press a key to close)", cv::WINDOW_NORMAL);
                                cv::imshow("FreeTure (Press a key to close)", temp);
                                cv::waitKey(0);

                            }
                        }
    }
    catch(exception& ex)
    {
        LOG_ERROR << "Freeture::modeSingleAcquisition Exception" << ex.what();
    }
    delete device;
}

/// <summary>
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%% MODE 5 : CLEAN LOGS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/// Run freeture as Clean logs
/// </summary>
void freeture::Freeture::modeCleanLogs()
{
    LOG_INFO << "================================================";
    LOG_INFO << "======== FREETURE - Clean logs         =========";
    LOG_INFO << "================================================" << endl;

    // Simply remove all log directory contents.
    boost::filesystem::path p(m_FreetureSettings->getLogParam().LOG_PATH);

    if(boost::filesystem::exists(p)){
        boost::filesystem::remove_all(p);
        LOG_INFO << "Clean log completed." ;
    }else {
        LOG_INFO << "Log directory not found." ;
    }
}

/// <summary>
/// Main freeture routine function, routes program options to the correct operative function
/// </summary>
void freeture::Freeture::Run()
{
    fetchProgramOption();

    switch(m_CurrentRunMode)
    {
        case freeture::Mode::PRINT_VERSION:
        {
            printVersion();
            break;
        }
        case freeture::Mode::LIST_FORMATS:
        {
            selectListFormats();
            break;
        }
        case freeture::Mode::LIST_DEVICES:
        {
            selectListDevices();
            break;
        }
        case freeture::Mode::PRINT_HELP:
        {
            printHelp();
            break;
        }
        case freeture::Mode::CONTINUOUS_ACQUISITION  :
        {
            modeContinuousAcquisition();
            break;
        }
        case freeture::Mode::TEST_CONFIGURATION :
        {
            modeTest();
            break;
        }
        case freeture::Mode::METEOR_DETECTION  :
        {
            modeMeteorDetection();
            break;
        }
        case freeture::Mode::SINGLE_ACQUISITION :
        {
            modeSingleAcquisition();
            break;
        }
        case freeture::Mode::CLEAN_LOGS :
        {
            modeCleanLogs();
            break;
        }
    }
}
