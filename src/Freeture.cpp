//header refactoring ok
#include "Freeture.h"

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
    LOG_WARNING << "Received signal : " << signum;
    sigTermFlag = true;
}

freeture::Freeture::Freeture(int argc, const char ** argv)
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
    m_CurrentMode = Mode::UNKNOWN;

    switch(mode)
    {
        case 1:
        {
            m_CurrentMode = Mode::TEST_CONFIGURATION;
        }
        break;
        case 2 :
        {
            m_CurrentMode = Mode::CONTINUOUS_ACQUISITION;
        }
        break;
        case 3:
        {
            m_CurrentMode = Mode::METEOR_DETECTION;
        }
        break;
        case 4 :
        {
            m_CurrentMode = Mode::SINGLE_ACQUISITION;
        }
        break;
        case 5:
        {
            m_CurrentMode = Mode::CLEAN_LOGS;
        }
        break;
    }

    if (
         m_CurrentMode == Mode::METEOR_DETECTION ||
         m_CurrentMode == Mode::SINGLE_ACQUISITION ||
         m_CurrentMode == Mode::CONTINUOUS_ACQUISITION
    )
    {
        
            // Cam id.
            if(vm.count("id")) devID = vm["id"].as<int>();

            // Path where to save files.
            if(vm.count("savepath")) savePath = vm["savepath"].as<string>();

            // Acquisition pixel format.
            if(vm.count("format")) acqFormat = vm["format"].as<int>();

            // Crop start x
            if(vm.count("startx")) startx = vm["startx"].as<int>();
            // Crop start y
            if(vm.count("starty")) starty = vm["starty"].as<int>();
            // Cam width size
            if(vm.count("width")) acqWidth = vm["width"].as<int>();
            // Cam height size
            if(vm.count("height")) acqHeight = vm["height"].as<int>();
            // Gain value.
            if(vm.count("gain")) gain = vm["gain"].as<int>();
            // Exposure value.
            if(vm.count("exposure")) exp = vm["exposure"].as<double>();
            // Filename.
            if(vm.count("filename")) fileName = vm["filename"].as<string>();

            if(vm.count("display")) display = true;

            if(vm.count("bmp")) bmp = true;
            if(vm.count("fits")) fits = true;

            // Send fits by mail if configuration file is correct.
            if(vm.count("sendbymail")) sendbymail = true;
    }

    if (m_CurrentMode == freeture::Mode::UNKNOWN)
    {
        cout << "MODE " << mode << " is not available. Correct modes are : " << endl<<endl;
        cout << "[1] Check configuration file."                         << endl;
        cout << "[2] Run continous acquisition."                        << endl;
        cout << "[3] Run meteor detection."                             << endl;
        cout << "[4] Run single capture."                               << endl;
        cout << "[5] Clean logs."                                       << endl << endl;

        cout << "Execute freeture command to see options." << endl;
    }
}

/// <summary>
/// Print out version
/// </summary>
void freeture::Freeture::printVersion()
{
    cout << "Current version : " << string(VERSION) << endl;
}

/// <summary>
/// Print help
/// </summary>
void freeture::Freeture::printHelp()
{
    cout << *desc;
}

/// <summary>
/// Call Device list devices discovery
/// </summary>
void freeture::Freeture::selectListDevices()
{
   device = new Device();
   device->listDevices(true);
   delete device;
}

/// <summary>
/// Filter devices by pixel format
/// </summary>
void freeture::Freeture::selectListFormats()
{
    device = new Device();
    device->listDevices(false);

    if(device->createCamera(devID, true))
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
            m_CurrentMode = freeture::Mode::PRINT_VERSION;
        }
        else if(vm.count("help"))
        {
            m_CurrentMode = freeture::Mode::PRINT_HELP;
        }
        else if(vm.count("listdevices"))
        {
            m_CurrentMode = freeture::Mode::LIST_DEVICES;
        }
        else if(vm.count("listformats"))
        {
            m_CurrentMode = freeture::Mode::LIST_FORMATS;

            if(vm.count("id"))
                devID = vm["id"].as<int>();
        }
        else if(vm.count("mode"))
        {
            mode = vm["mode"].as<int>();

            if(vm.count("cfg")) configPath = vm["cfg"].as<string>();
            if(vm.count("time")) executionTime = vm["time"].as<int>();

            selectMode(vm);
        }
        else
        {
            m_CurrentMode = freeture::Mode::PRINT_HELP;
        }
    }
    catch(exception& e)
    {
        m_Error=true;
        cout << ">> Error : " << e.what() << endl;
    }

    po::notify(vm);
}

/// <summary>
/// Test the configuration file operative mode
/// </summary>
void freeture::Freeture::modeTest()
{
    device = new Device();
    CfgParam cfg(device, configPath);

    ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ///%%%%%%%%%%%% MODE 1 : TEST/CHECK CONFIGURATION FILE %%%%%%%%%%%%%%%%%%%
    ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    cout << "================================================" << endl;
    cout << "====== FREETURE - Test/Check configuration =====" << endl;
    cout << "================================================" << endl << endl;
    cfg.showErrors = true;
    cfg.allParamAreCorrect();
    delete device;
}

/// <summary>
/// Continue acquisition mode
/// </summary>
void freeture::Freeture::modeContinuousAcquisition()
{
    ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ///%%%%%%%%%%%%%%%% MODE 2 : CONTINUOUS ACQUISITION %%%%%%%%%%%%%%%%%%%%%%
    ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    cout << "================================================" << endl;
    cout << "========== FREETURE - Continuous mode ==========" << endl;
    cout << "================================================" << endl << endl;

    EParser<CamPixFmt> fmt;
    string fstring = fmt.getStringEnum(static_cast<CamPixFmt>(acqFormat));
    if(fstring == "")
        throw ">> Pixel format specified not found.";

    cout << "------------------------------------------------" << endl;
    cout << "CAM ID    : " << devID << endl;
    cout << "FORMAT    : " << fstring << endl;
    cout << "GAIN      : " << gain << endl;
    cout << "EXPOSURE  : " << exp << endl;
    cout << "------------------------------------------------" << endl << endl;


    device->listDevices(false);
    device->mFormat = static_cast<CamPixFmt>(acqFormat);

    if(!device->createCamera(devID, true)) {
        throw "Fail to create device.";
    }

    if(acqWidth != 0 && acqHeight != 0)
        device->setCameraSize(startx, starty, acqWidth, acqHeight);
    else
        device->setCameraSize();

    if(!device->setCameraPixelFormat()) {
        throw "Fail to set format";
    }

    device->setCameraFPS();
    device->setCameraExposureTime(exp);
    device->setCameraGain(gain);
    device->initializeCamera();
    device->startCamera();

    if(display)
        cv::namedWindow("FreeTure (ESC to stop)", cv::WINDOW_NORMAL);


    char hitKey;
    int interruption = 0;

    while(!interruption)
    {
                            Frame frame;

                            double tacq = (double)cv::getTickCount();
                            if(device->runContinuousCapture(frame)){
                                tacq = (((double)cv::getTickCount() - tacq)/ cv::getTickFrequency())*1000;
                                cout << " >> [ TIME ACQ ] : " << tacq << " ms" << endl;

                                if(display) {
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

void freeture::Freeture::modeMeteorDetection()
{
    CameraDeviceManager& manager = CameraDeviceManager::Get();
    device = manager.getDevice();
    //manager.listDevice();

    CfgParam cfg(device, configPath);
    ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ///%%%%%%%%%%%%%%%%%%%% MODE 3 : METEOR DETECTION %%%%%%%%%%%%%%%%%%%%%%%%
    ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    cout << "================================================" << endl;
    cout << "======= FREETURE - Meteor detection mode =======" << endl;
    cout << "================================================" << endl << endl;

   /// ------------------------------------------------------------------
   /// --------------------- LOAD FREETURE PARAMETERS -------------------
   /// ------------------------------------------------------------------

   cfg.showErrors = true;
   if(!cfg.allParamAreCorrect())
        throw "Configuration file is not correct. Fail to launch detection mode.";

    device->Setup(cfg.getCamParam(), cfg.getFramesParam(), cfg.getVidParam(), devID);


    vector<string> logFiles;
    logFiles.push_back("MAIN_THREAD.log");
    logFiles.push_back("ACQ_THREAD.log");
    logFiles.push_back("DET_THREAD.log");
    logFiles.push_back("STACK_THREAD.log");

    //device->Setup(cfg.getCamParam(), cfg.getFramesParam(), cfg.getVidParam(), devID);

    //Logger logSystem(cfg.getLogParam().LOG_PATH, cfg.getLogParam().LOG_ARCHIVE_DAY, cfg.getLogParam().LOG_SIZE_LIMIT, logFiles);
    
    /// ------------------------------------------------------------------
    /// -------------------------- MANAGE LOG ----------------------------
    /// ------------------------------------------------------------------
    fs::path pLog(cfg.getLogParam().LOG_PATH);
    if(!fs::exists(pLog))
    {

                            if(!fs::create_directory(pLog))
                                throw "> Failed to create a directory for logs files.";
                            else
                                cout << "> Log directory created : " << pLog << endl;
    }

    
    BOOST_LOG_SCOPED_THREAD_TAG("LogName", "MAIN_THREAD");

    LOG_INFO << "\n";
    LOG_INFO << "==============================================";
    LOG_INFO << "====== FREETURE- Meteor detection mode ======";
    LOG_INFO << "==============================================";

    /// ------------------------------------------------------------------
    /// ------------------------- SHARED RESSOURCES ----------------------
    /// ------------------------------------------------------------------
   
    // Circular buffer to store last n grabbed frames.
    boost::circular_buffer<Frame> frameBuffer(cfg.getDetParam().ACQ_BUFFER_SIZE * cfg.getCamParam().ACQ_FPS);
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

    AcqThread   *acqThread      = nullptr;
    DetThread   *detThread      = nullptr;
    StackThread *stackThread    = nullptr;

    try {

                            // Create detection thread.
                            if(cfg.getDetParam().DET_ENABLED) {

                                LOG_INFO << "Start to create detection thread.";
                                cout << "Start to create detection thread." << endl;

                                detThread = new DetThread(  &frameBuffer,
                                                            &frameBuffer_m,
                                                            &frameBuffer_c,
                                                            &signalDet,
                                                            &signalDet_m,
                                                            &signalDet_c,
                                                            cfg.getDetParam(),
                                                            cfg.getDataParam(),
                                                            cfg.getMailParam(),
                                                            cfg.getStationParam(),
                                                            cfg.getFitskeysParam(),
                                                            cfg.getCamParam().ACQ_FORMAT);

                                if(!detThread->startThread())
                                    throw "Fail to start detection thread.";

                            }

                            

                            // Create stack thread.
                            if(cfg.getStackParam().STACK_ENABLED) {

                                LOG_INFO << "Start to create stack Thread.";

                                stackThread = new StackThread(  &signalStack,
                                                                &signalStack_m,
                                                                &signalStack_c,
                                                                &frameBuffer,
                                                                &frameBuffer_m,
                                                                &frameBuffer_c,
                                                                cfg.getDataParam(),
                                                                cfg.getStackParam(),
                                                                cfg.getStationParam(),
                                                                cfg.getCamParam().ACQ_FORMAT,
                                                                cfg.getFitskeysParam());

                                if(!stackThread->startThread())
                                    throw "Fail to start stack thread.";

                            }

                            
                            
                            // Create acquisition thread.
                            acqThread = new AcqThread(  &frameBuffer,
                                                        &frameBuffer_m,
                                                        &frameBuffer_c,
                                                        &signalStack,
                                                        &signalStack_m,
                                                        &signalStack_c,
                                                        &signalDet,
                                                        &signalDet_m,
                                                        &signalDet_c,
                                                        detThread,
                                                        stackThread,
                                                        cfg.getDeviceID(),
                                                        cfg.getDataParam(),
                                                        cfg.getStackParam(),
                                                        cfg.getStationParam(),
                                                        cfg.getDetParam(),
                                                        cfg.getCamParam(),
                                                        cfg.getFramesParam(),
                                                        cfg.getVidParam(),
                                                        cfg.getFitskeysParam(),
                                                        manager.getDevice());

                            if(!acqThread->startThread()) {

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

                                while(!sigTermFlag && !interruption) {


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
                                    if(executionTime != 0) {

                                        if(cptTime > executionTime){
                                            break;
                                        }

                                        cptTime ++;

                                    }

                                    /// Stop freeture if one of the thread is stopped.
                                    if(acqThread->getThreadStatus()){
                                        break;
                                    }

                                    if(detThread != NULL){
                                        if(!detThread->getRunStatus()){
                                            LOG_ERROR << "DetThread not running. Stopping the process ...";
                                            break;
                                        }
                                    }

                                    if(stackThread != NULL){
                                        if(!stackThread->getRunStatus()){
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

                        if(acqThread != NULL) {
                            acqThread->stopThread();
                            delete acqThread;
                        }

                        if(detThread != NULL) {
                            detThread->stopThread();
                            delete detThread;
                        }

                        if(stackThread != NULL){
                            stackThread->stopThread();
                            delete stackThread;

                        }

}

/// <summary>
/// Run freeture as single acquisition
/// </summary>
void freeture::Freeture::modeSingleAcquisition()
{
    CameraDeviceManager& manager = CameraDeviceManager::Get();
    device = manager.getDevice();

    try
    {
        CfgParam cfg(device, configPath);
        ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ///%%%%%%%%%%%%%%%%%%% MODE 4 : SINGLE ACQUISITION %%%%%%%%%%%%%%%%%%%%%%%
        ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        device->Setup(cfg.getCamParam(), cfg.getFramesParam(), cfg.getVidParam(), devID);

        cout << "================================================" << endl;
        cout << "======== FREETURE - Single acquisition =========" << endl;
        cout << "================================================" << endl << endl;

        EParser<CamPixFmt> fmt;
        string fstring = fmt.getStringEnum(static_cast<CamPixFmt>(acqFormat));
        if(fstring == "")
            throw ">> Pixel format specified not found.";

        cout << "------------------------------------------------" << endl;
        cout << "CAM ID    : " << devID << endl;
        cout << "FORMAT    : " << fstring << endl;
        cout << "GAIN      : " << gain << endl;
        cout << "EXPOSURE  : " << exp << endl;

        if(acqWidth > 0 && acqHeight > 0) cout << "SIZE      : " << acqWidth << "x" << acqHeight << endl;
        if(startx > 0 || starty > 0) cout << "START X,Y : " << startx << "," << starty << endl;

        cout << "SAVE PATH : " << savePath << endl;
        cout << "FILENAME  : " << fileName << endl;
        cout << "------------------------------------------------" << endl << endl;

        /// ------------------------------------------------------------------
        /// ----------------------- MANAGE FILE NAME -------------------------
        /// ------------------------------------------------------------------

        int filenum = 0;
        bool increment = false;
        path p(savePath);

        // Search previous captures in the directory.
        for(directory_iterator file(p);file!= directory_iterator(); ++file)
        {

                            path curr(file->path());

                            if(is_regular_file(curr)) {

                                list<string> ch;
                                string fname = curr.filename().string();
                                Conversion::stringTok(ch, fname.c_str(), "-");

                                int i = 0;
                                int n = 0;

                                if(ch.front() == fileName && ch.size() == 2) {

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
                        frame.mExposure = exp;
                        frame.mGain = gain;
                        frame.mFormat = static_cast<CamPixFmt>(acqFormat);
                        frame.mStartX = startx != 0 ? startx : devStartx;
                        frame.mStartY = starty != 0 ? starty : devStarty;
                        frame.mHeight = acqHeight != 0 ? acqHeight : devHeight;
                        frame.mWidth = acqWidth != 0 ? acqWidth : devWidth;

                        cout << "++++++++++++++++++++++++++ startx " << frame.mStartX << endl;
                        cout << "++++++++++++++++++++++++++ starty " << frame.mStartY << endl;
                        cout << "++++++++++++++++++++++++++ acqHeight " << frame.mHeight << endl;
                        cout << "++++++++++++++++++++++++++ acqWidth " << frame.mWidth << endl;


                       
                        device->setCameraAutoExposure(false);
                        device->setCameraGain(gain);
                        device->setCameraExposureTime(exp);

                        

                        if(!device->runSingleCapture(frame)){
                            throw ">> Single capture failed.";
                        }


                        cout << ">> Single capture succeed." << endl;

                        /// ------------------------------------------------------------------
                        /// ------------------- SAVE / DISPLAY CAPTURE -----------------------
                        /// ------------------------------------------------------------------

                        if(frame.mImg.data) {

                            // Save the frame in BMP.
                            if(bmp) {

                                cout << ">> Saving bmp file ..." << endl;

                                cv::Mat temp1, newMat;
                                frame.mImg.copyTo(temp1);

                                if(frame.mFormat == MONO12){
                                    newMat = ImgProcessing::correctGammaOnMono12(temp1, 2.2);
                                    cv::Mat temp = Conversion::convertTo8UC1(newMat);
                                }else {
                                    newMat = ImgProcessing::correctGammaOnMono8(temp1, 2.2);
                                }

                                SaveImg::saveBMP(newMat, savePath + fileName + "-" + Conversion::intToString(filenum));
                                cout << ">> Bmp saved : " << savePath << fileName << "-" << Conversion::intToString(filenum) << ".bmp" << endl;

                            }

                            // Save the frame in Fits 2D.
                            if(fits) {

                                cout << ">> Saving fits file ..." << endl;

                                Fits fh;
                                bool useCfg = false;
                                Fits2D newFits(savePath);

                                cfg.showErrors = true;
                                if(cfg.stationParamIsCorrect() && cfg.fitskeysParamIsCorrect()) {
                                    useCfg = true;
                                    double  debObsInSeconds = frame.mDate.hours*3600 + frame.mDate.minutes*60 + frame.mDate.seconds;
                                    double  julianDate      = TimeDate::gregorianToJulian(frame.mDate);
                                    double  julianCentury   = TimeDate::julianCentury(julianDate);
                                    double  sideralT        = TimeDate::localSideralTime_2(julianCentury, frame.mDate.hours, frame.mDate.minutes, (int)frame.mDate.seconds, fh.kSITELONG);
                                    newFits.kCRVAL1 = sideralT;
                                    newFits.loadKeys(cfg.getFitskeysParam(), cfg.getStationParam());

                                }

                                newFits.kGAINDB = (int)gain;
                                newFits.kELAPTIME = exp/1000000.0;
                                newFits.kEXPOSURE = exp/1000000.0;
                                newFits.kONTIME = exp/1000000.0;
                                newFits.kDATEOBS = TimeDate::getIsoExtendedFormatDate(frame.mDate);
                                newFits.kCTYPE1 = "RA---ARC";
                                newFits.kCTYPE2 = "DEC--ARC";
                                newFits.kEQUINOX = 2000.0;

                                if(frame.mFormat == MONO12){
                                    // Create FITS image with BITPIX = SHORT_IMG (16-bits signed integers), pixel with TSHORT (signed short)
                                    if(newFits.writeFits(frame.mImg, S16, fileName + "-" + Conversion::intToString(filenum)))
                                        cout << ">> Fits saved in : " << savePath << fileName << "-" << Conversion::intToString(filenum) << ".fit" << endl;
                                }else{
                                    // Create FITS image with BITPIX = BYTE_IMG (8-bits unsigned integers), pixel with TBYTE (8-bit unsigned byte)
                                    if(newFits.writeFits(frame.mImg, UC8, fileName + "-" + Conversion::intToString(filenum)))
                                        cout << ">> Fits saved in : " << savePath << fileName << "-" << Conversion::intToString(filenum) << ".fit" << endl;

                                }

                                // Send fits by mail if configuration file is correct.
                                if(sendbymail && useCfg) {

                                    if(cfg.mailParamIsCorrect()) {

                                        vector<string> mailAttachments;
                                        mailAttachments.push_back(savePath + fileName + "-" + Conversion::intToString(filenum) + ".fit");

                                        SMTPClient::sendMail(cfg.getMailParam().MAIL_SMTP_SERVER,
                                                            cfg.getMailParam().MAIL_SMTP_LOGIN,
                                                            cfg.getMailParam().MAIL_SMTP_PASSWORD,
                                                            "freeture@snap",
                                                            cfg.getMailParam().MAIL_RECIPIENTS,
                                                            fileName + "-" + Conversion::intToString(filenum) + ".fit",
                                                            " Exposure time : " + Conversion::intToString((int)exp) + "\n Gain : " + Conversion::intToString((int)gain),
                                                            mailAttachments,
                                                            cfg.getMailParam().MAIL_CONNECTION_TYPE);

                                    }
                                }

       }

                            // Display the frame in an opencv window
                            if(display) {

                                cout << ">> Display single capture." << endl;

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
/// Run freeture as Clean logs
/// </summary>
void freeture::Freeture::modeCleanLogs()
{
    device = new Device();
    CfgParam cfg(device, configPath);
    ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ///%%%%%%%%%%%%%%%%%%%%%% MODE 5 : CLEAN LOGS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    // Simply remove all log directory contents.
    boost::filesystem::path p(cfg.getLogParam().LOG_PATH);

    if(boost::filesystem::exists(p)){
        boost::filesystem::remove_all(p);
        cout << "Clean log completed." << endl;
    }else {
        cout << "Log directory not found." << endl;
    }
    delete device;
}

/// <summary>
/// Main freeture routine function, routes program options to the correct operative function
/// </summary>
void freeture::Freeture::Run()
{
    fetchProgramOption();

    switch(m_CurrentMode)
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
