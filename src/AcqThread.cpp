/*
                            AcqThread.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2016 Yoan Audureau, Chiara Marmo
*                               GEOPS-UPSUD-CNRS
*
*   License:        GNU General Public License
*
*   FreeTure is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*   FreeTure is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*   You should have received a copy of the GNU General Public License
*   along with FreeTure. If not, see <http://www.gnu.org/licenses/>.
*
*   Last modified:      03/10/2016
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/*
 *
 *  This code is designed for the automated detection and capture of transient events in the night sky,
 *  likely with a focus on meteor detection. The file AcqThread.cpp contains the implementation of an
 *  acquisition thread responsible for managing the capture of images from a camera device, including
 *  setting the camera in continuous mode, managing the detection and stacking of images, and adjusting
 *  camera settings based on environmental conditions (e.g., day/night).
 *
 *  Here's a brief overview of the main functionalities and components found in this file:
 *
 *      * Constructor and Destructor: Initializes and cleans up resources used by the acquisition thread.
 *      This includes setting up links to other threads (DetThread for detection, StackThread for image stacking),
 *      and configuring camera settings.
 *
 *      * Thread Management: Includes methods to start and stop the acquisition thread, and checks the thread's
 *      status. It ensures proper synchronization between threads using mutexes and condition variables.
 *
 *      * Camera Settings and Modes: Functions to configure the camera for continuous acquisition, including
 *      setting up the camera's exposure, gain, and pixel format based on the time of day and specific user
 *      configurations (e.g., day/night settings).
 *
 *      * Image Acquisition and Processing: Manages the capture of images in a loop, processing each frame according
 *      to the current environmental conditions and the configured detection and stacking parameters.
 *      This includes handling different input types (e.g., live camera feed, video files, image sequences)
 *      and adjusting for sunrise and sunset times to switch between different capture modes.
 *
 *      * Utility Functions: Includes a range of helper functions to support the main functionalities,
 *      such as computing sunrise and sunset times, managing the acquisition schedule, saving captured images
 *      in different formats, and creating directories for data storage.
 *
 *  Overall, the AcqThread.cpp file is a critical component of the FreeTure system, handling the complex task
 *  of continuously acquiring images from a camera, adjusting settings in real-time based on environmental changes,
 *  and coordinating with other components of the system for the detection and stacking of transient events.
 *
 */
#include "AcqThread.h"
#include <memory>

#include "DetThread.h"
#include "StackThread.h"

#include "CfgParam.h"

#include "Detection.h"
#include "NodeExporterMetrics.h"
#include "CameraDeviceManager.h"
#include "Device.h"
#include "ExposureControl.h"
#include "Ephemeris.h"
#include "Fits2D.h"
#include "SaveImg.h"
#include "Conversion.h"
#include "ImgProcessing.h"
#include "Camera.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

using namespace std;
using namespace freeture;

AcqThread::AcqThread(boost::circular_buffer<shared_ptr<Frame>>& fb,
    boost::mutex* fb_m,
    boost::condition_variable* fb_c,
    bool* sSignal,
    boost::mutex* sSignal_m,
    boost::condition_variable* sSignal_c,
    bool* dSignal,
    boost::mutex* dSignal_m,
    boost::condition_variable* dSignal_c,
    shared_ptr<DetThread>               detection,
    shared_ptr<StackThread>             stack,
    std::shared_ptr<CfgParam> cfg
) :
    m_ThreadID(),
    m_CurrentThreadLoopTime(),
    m_PreviousThreadLoopTime(),
    mMustStopMutex(),
    mThread(nullptr),
    mMustStop(false), 
    mThreadTerminated(false),
    frameBuffer_condition(fb_c),
    frameBuffer_mutex(fb_m),
    frameBuffer(fb),
    m_StackThread_IsRunning(false),
    stackSignal(sSignal),
    stackSignal_mutex(sSignal_m),
    stackSignal_condition(sSignal_c),
    m_DetThread_IsRunning(false),
    detSignal(dSignal),
    detSignal_mutex(dSignal_m),
    detSignal_condition(dSignal_c),
    m_Device(nullptr),
    pExpCtrl(nullptr),
    pDetection(detection),
    pStack(stack),
    mOutputDataPath(""),
    mCurrentDate(""),
    m_LastRegularAcquisitionTimestamp(),
    m_LastMetricTimestamp(),
    mDeviceID(0),
    mNextAcqIndex(0),
    mStartSunriseTime(0),
    mStopSunriseTime(0),
    mStartSunsetTime(0),
    mStopSunsetTime(0),
    mCurrentTime(0),
    mFrameNumber(0),
    cleanStatus(false),
    mNextAcq(),
    m_StackParam(cfg->getStackParam()),
    m_StationParam(cfg->getStationParam()),
    m_DetectionParam(cfg->getDetParam()),
    m_DataParam(cfg->getDataParam()),
    mfkp(cfg->getFitskeysParam()),
    m_CameraParam(cfg->getCamParam()),
    m_FramesParam(cfg->getFramesParam()),
    m_VideoParam(cfg->getVidParam()),
    m_CurrentCameraSettings(),
    m_CurrentAcquisitionMode(),
    m_FPS_metrics(cfg->getDetParam().ACQ_BUFFER_SIZE* cfg->getCamParam().ACQ_FPS),
    m_CurrentFPS(0.0),
    tacq_metric(0.0),
    m_FPS_Sum(0.0),
    nextSunset(),
    nextSunrise(),
    m_CircularBufferSize(cfg->getDetParam().ACQ_BUFFER_SIZE* cfg->getCamParam().ACQ_FPS),
    m_CurrentTimeMode(),
    m_CurrentTimeInSeconds(0),
    m_CurrentFrame(),
    previousTimeMode(),
    time_mode_parser(),
    m_RunnedRegularAcquisition(false),
    m_RunnedScheduledAcquisition(false),
    exposureControlStatus(false),
    exposureControlActive(false)
  
{
    LOG_DEBUG << "AcqThread::AcqThread";
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    m_Device                 = m_CameraDeviceManager->getDevice();
    
}

AcqThread::~AcqThread(void){
    LOG_DEBUG << "AcqThread::~AcqThread"<< endl;
    if(m_Device != NULL)
        delete m_Device;

    if(mThread != NULL)
        delete mThread;

    if(pExpCtrl != NULL)
        delete pExpCtrl;

}

void AcqThread::stopThread(){
    LOG_DEBUG << "AcqThread::stopThread";

    mMustStopMutex.lock();
    mMustStop = true;
    mMustStopMutex.unlock();

    if(mThread != NULL)
        while(mThread->timed_join(boost::posix_time::seconds(2)) == false)
            mThread->interrupt();

}

bool AcqThread::setCameraSingleFrameMode(EAcquisitionMode mode)
{

    LOG_DEBUG << "AcqThread::setCameraSingleMode";
    if (m_Device->isStreaming())
        m_Device->stopCamera();

    m_Device->Setup(m_CameraParam, m_FramesParam, m_VideoParam);

    LOG_DEBUG << "AcqThread::setCameraSingleMode;" << "Prepare single-frame acquisition.";

    if (!prepareAcquisitionOnDevice(mode)) {
        LOG_DEBUG << "FAIL ON PREPARE AQ DEVICE";
        return false;
    }

    m_CurrentAcquisitionMode = mode;

    return true;
}

/// <summary>
/// WHERE IS CONTINUOUS MODE??
/// </summary>
/// <param name="rebuild"></param>
/// <returns></returns>
bool AcqThread::setCameraInContinousMode()
{
    LOG_DEBUG << "AcqThread::setCameraInContinousMode";

    if (m_Device->isStreaming())
        m_Device->stopCamera();

    m_Device->Setup(m_CameraParam, m_FramesParam, m_VideoParam);

    if(!prepareAcquisitionOnDevice(EAcquisitionMode::CONTINUOUS)) {
        LOG_DEBUG << "FAIL ON PREPARE AQ DEVICE";
        return false;
    }

    m_CurrentAcquisitionMode = EAcquisitionMode::CONTINUOUS;

    return true;
}  

bool AcqThread::startThread()
{
    LOG_DEBUG << "AcqThread::startThread";
    
    // Create acquisition thread.
    mThread = new boost::thread(boost::ref(*this));
    
    return true;
}

bool AcqThread::getThreadStatus(){
    
    return mThreadTerminated;
}

void AcqThread::startDetectionThread() 
{
    if (pDetection != nullptr) {
        LOG_DEBUG << "AcqThread::startDetectionThread; DETECTION THREAD START SIGNAL";
        boost::mutex::scoped_lock lock2(*detSignal_mutex);
        *detSignal = true;
        detSignal_condition->notify_one();
        lock2.unlock();
    }
}


void AcqThread::stopDetectionThread()
{
    if ( pDetection != nullptr ) {
        LOG_DEBUG << "AcqThread::stopDetectionThread; DETECTION THREAD STOP SIGNAL";

        boost::mutex::scoped_lock lock(*detSignal_mutex);
        *detSignal = false;
        lock.unlock();
        
        // Force interruption.
        LOG_DEBUG << "AcqThread::stopDetectionThread;" << "Sending interruption signal to detection process... ";
        pDetection->interruptThread();
    }
}

void AcqThread::startStackThread()
{
    if (pStack != nullptr)
    {
        LOG_DEBUG << "AcqThread::startStackThread; STACK THREAD START SIGNAL";

        boost::mutex::scoped_lock lock3(*stackSignal_mutex);
        *stackSignal = true;
        stackSignal_condition->notify_one();
        lock3.unlock();
    }
}


void AcqThread::stopStackThread()
{
    if (pStack != nullptr)
    {
        LOG_DEBUG << "AcqThread::stopStackThread; STACK THREAD STOP SIGNAL";

        boost::mutex::scoped_lock lock(*stackSignal_mutex);
        *stackSignal = false;
        lock.unlock();

        // Force interruption.
        LOG_DEBUG <<"AcqThread::stopStackThread;" << "Send interruption signal to stack ";
        pStack->interruptThread();
    }
}

void AcqThread::resetFrameBuffer()
{
    LOG_DEBUG << "AcqThread::resetFrameBuffer;" << "Cleaning frameBuffer...";
    boost::mutex::scoped_lock lock(*frameBuffer_mutex);

    frameBuffer.clear();

    mFrameNumber = 0;

    lock.unlock();

    cleanStatus = true;
}

void AcqThread::operator()()
{
    // Obtain the attribute_value which holds the thread ID
    m_ThreadID = std::this_thread::get_id();

    Logger::GetLogger()->setLogThread(LogThread::ACQUISITION_THREAD, m_ThreadID, true);

    bool stop = false;


    LOG_INFO << "AcqThread::operator();" << "==============================================";
    LOG_INFO << "AcqThread::operator();" << "========== START ACQUISITION THREAD ==========";
    LOG_INFO << "AcqThread::operator();" << "==============================================";

    try {
        // Search next acquisition according to the current time.
        selectNextAcquisitionSchedule(TimeDate::splitIsoExtendedDate(boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time())));

   
        // If exposure can be set on the input device.
        if (m_DetectionParam.ACQ_AUTOEXPOSURE_ENABLED)
        {
            if (m_Device->getExposureStatus())
            {
                if (pExpCtrl != nullptr)
                    delete pExpCtrl;

                else pExpCtrl = new ExposureControl(m_CameraParam.EXPOSURE_CONTROL_FREQUENCY,
                    m_CameraParam.EXPOSURE_CONTROL_SAVE_IMAGE,
                    m_CameraParam.EXPOSURE_CONTROL_SAVE_INFOS,
                    m_DataParam.DATA_PATH,
                    m_StationParam.STATION_NAME);
            }
        }

        previousTimeMode = getTimeMode(getNowInSeconds());
       
        LOG_INFO << "CURRENT TIME MODE: " << time_mode_parser.getStringEnum(previousTimeMode);
        //Initialize low frequency parameters
        m_CurrentCameraSettings.StartX = m_CameraParam.ACQ_STARTX;
        m_CurrentCameraSettings.StartY = m_CameraParam.ACQ_STARTY;
        m_CurrentCameraSettings.SizeWidth = m_CameraParam.ACQ_WIDTH;
        m_CurrentCameraSettings.SizeHeight = m_CameraParam.ACQ_HEIGHT;

        /// Acquisition process.
        do {
            InputDeviceType input_device_type = m_Device->getDeviceType();

            if (input_device_type == InputDeviceType::VIDEO || input_device_type == InputDeviceType::SINGLE_FITS_FRAME)
            {
                // Location of a video or frames if input type is FRAMES or VIDEO.
                string location = "";

                // Load videos file or frames directory if input type is FRAMES or VIDEO
                if (!m_Device->loadNextCameraDataSet(location))
                    break;

                if (pDetection != nullptr)
                    pDetection->setCurrentDataSet(location);
            }

            // Reference time to compute interval between regular captures.
            m_PreviousThreadLoopTime = m_LastRegularAcquisitionTimestamp = m_CurrentThreadLoopTime = boost::posix_time::microsec_clock::universal_time();

            if (!setCameraInContinousMode()) {
                LOG_ERROR << "AcqThread::operator();" << "Cannot set continuous camera mode";
                break;
            }

          
           
            do
            {
                //WRITE CODE TO BE NOT CLEARED EVERY TIME OF THE DAY... USELESS CARBON 
                nextSunset.clear();
                nextSunrise.clear();


                m_CurrentThreadLoopTime = boost::posix_time::microsec_clock::universal_time();
                m_CurrentTimeDate = TimeDate::Date(m_CurrentThreadLoopTime);
                m_CurrentTimeInSeconds = m_CurrentTimeDate.hours * 3600 + m_CurrentTimeDate.minutes * 60 + (int)m_CurrentTimeDate.seconds;

                // Detect day or night.
                m_CurrentTimeMode = getTimeMode(m_CurrentTimeInSeconds);

                tacq_metric = (double)cv::getTickCount();

                //check if device is connected.
                if (!m_Device->isConnected()) {
                    LOG_ERROR << "DEVICE IS NOT CONNECTED ANYMORE";
                    if (!m_Device->connect()) {
                        LOG_ERROR << "CONNECTION FAILED WAITING 1s";
                        boost::this_thread::sleep(boost::posix_time::millisec(1000));
                        continue;
                    }
                }

                continuousAcquisition_ThreadLoop();

                regularAcquisition_ThreadLoop();

                scheduledAcquisition_ThreadLoop();


                //CALCULATE SUN EPHEMERYS

                // If the date has changed, sun ephemeris must be updated.
                if (m_PreviousThreadLoopTime.date() != boost::posix_time::microsec_clock::universal_time().date())
                {
                    LOG_INFO << "AcqThread::operator();" << "Date has changed. Former Date is " << m_PreviousThreadLoopTime.date() << ". New Date is " << m_CurrentThreadLoopTime.date() << ".";
                    computeSunTimes();
                }

                // Print time before sunrise.

                if (!isSunrise(m_CurrentTimeInSeconds)) {
                    nextSunrise.clear();
                    if (m_CurrentTimeInSeconds < mStartSunriseTime)
                        nextSunrise = TimeDate::HdecimalToHMS((mStartSunriseTime - m_CurrentTimeInSeconds) / 3600.0);
                    if (m_CurrentTimeInSeconds > mStopSunsetTime)
                        nextSunrise = TimeDate::HdecimalToHMS(((24 * 3600 - m_CurrentTimeInSeconds) + mStartSunriseTime) / 3600.0);
                    if (LOG_SPAM_FRAME_STATUS)
                        if (nextSunrise.size() > 0)
                            LOG_DEBUG << "AcqThread::operator();" << "NEXT SUNRISE : " << nextSunrise.at(0) << "h" << nextSunrise.at(1) << "m" << nextSunrise.at(2) << "s";
                }

                // Print time before sunset.
                if (!isSunset())
                {
                    nextSunset.clear();
                    nextSunset = TimeDate::HdecimalToHMS((mStartSunsetTime - m_CurrentTimeInSeconds) / 3600.0);
                    if (LOG_SPAM_FRAME_STATUS)
                        if (nextSunset.size() > 0)
                            LOG_DEBUG << "AcqThread::operator();" << "NEXT SUNSET : " << nextSunset.at(0) << "h" << nextSunset.at(1) << "m" << nextSunset.at(2) << "s";
                }


                //CALCULATE METRICS
                tacq_metric = ((static_cast<double>(cv::getTickCount()) - tacq_metric) / cv::getTickFrequency()) * 1000;
                m_CurrentFPS = (1.0 / (tacq_metric / 1000.0));

                metrics_ThreadLoop();

                m_RunnedScheduledAcquisition = false;
                m_RunnedRegularAcquisition = false;

                //CHECK THREAD STOP
                mMustStopMutex.lock();
                stop = mMustStop;
                mMustStopMutex.unlock();

                previousTimeMode = m_CurrentTimeMode;

            } while (stop == false && !m_Device->getCameraStatus());

            // Reset detection process to prepare the analyse of a new data set.
            if (pDetection != NULL)
            {
                pDetection->getDetMethod()->resetDetection(true);
                pDetection->getDetMethod()->resetMask();
                pDetection->updateDetectionReport();
                if (!pDetection->getRunStatus())
                    break;

            }

            // Clear framebuffer.
            resetFrameBuffer();

        } while (m_Device->getCameraDataSetStatus() && stop == false);

    }
    catch (const boost::thread_interrupted&) {
        LOG_WARNING << "AcqThread::operator();" << "AcqThread ended.";
    }
    catch (exception& e) {
        LOG_ERROR << "AcqThread::operator();" << "An exception occured : " << e.what();
    }
    catch (const char* msg) {
        LOG_ERROR << "AcqThread::operator();" << msg;
    }

    m_Device->stopCamera();

    mThreadTerminated = true;

    LOG_INFO << "AcqThread::operator();" << "Acquisition Thread TERMINATED";
}


void  AcqThread::runScheduledAcquisition(TimeDate::Date mDate) {
    LOG_INFO << "AcqThread::runScheduledAcquisition;\t\t" << "Run scheduled acquisition.";
    m_RunnedScheduledAcquisition = true;
    //reset framebuffer because time passed, free memory
    LOG_DEBUG << "AcqThread::runScheduledAcquisition;\t\t" << "Reset frame buffer";
    resetFrameBuffer();

    CamPixFmt format;
    format = mNextAcq.fmt;

    if (m_CurrentAcquisitionMode != EAcquisitionMode::SCHEDULED) {
        LOG_INFO << "AcqThread::runScheduledAcquisition;\t\t" << "Setting single-frame mode";
        setCameraSingleFrameMode(EAcquisitionMode::SCHEDULED);
    }

    LOG_INFO << "AcqThread::runScheduledAcquisition;\t\t" << "Taking shot";
    runImageCapture(EAcquisitionMode::SCHEDULED,
        mNextAcq.rep,
        m_CameraParam.schcap.ACQ_SCHEDULE_OUTPUT,
        "");

//     LOG_INFO << "AcqThread::runScheduledAcquisition;\t\t" << "Saving image";
//     saveImageCaptured(frame, 0, m_CameraParam.regcap.ACQ_REGULAR_OUTPUT, m_CameraParam.regcap.ACQ_REGULAR_PRFX + "_reg");

    // Update mNextAcq
    selectNextAcquisitionSchedule(mDate);
}



void  AcqThread::runRegularAcquisition()
{
    LOG_INFO << "AcqThread::runRegularAcquisition;\t\t" << "Run regular acquisition.";
    m_RunnedRegularAcquisition = true;
    //reset framebuffer because time passed, free memory
    LOG_DEBUG << "AcqThread::runRegularAcquisition;\t\t" << "Reset frame buffer";
    resetFrameBuffer();

    if (m_CurrentAcquisitionMode != EAcquisitionMode::REGULAR) {
        LOG_INFO << "AcqThread::runRegularAcquisition;\t\t" << "Setting single-frame mode";
        setCameraSingleFrameMode(EAcquisitionMode::REGULAR);
    }

    LOG_INFO << "AcqThread::runRegularAcquisition;\t\t" << "Taking shot";
    runImageCapture(EAcquisitionMode::REGULAR,
        m_CameraParam.regcap.ACQ_REGULAR_CFG.rep,
        m_CameraParam.regcap.ACQ_REGULAR_OUTPUT,
        m_CameraParam.regcap.ACQ_REGULAR_PRFX);
}



void AcqThread::selectNextAcquisitionSchedule(TimeDate::Date date) {
    LOG_DEBUG << "AcqThread::selectNextAcquisitionSchedule";

    if(m_CameraParam.schcap.ACQ_SCHEDULE.size() != 0){

        // Search next acquisition
        for(int i = 0; i < m_CameraParam.schcap.ACQ_SCHEDULE.size(); i++){

            if(date.hours < m_CameraParam.schcap.ACQ_SCHEDULE.at(i).hours){

               mNextAcqIndex = i;
               break;

            }else if(date.hours == m_CameraParam.schcap.ACQ_SCHEDULE.at(i).hours){

                if(date.minutes < m_CameraParam.schcap.ACQ_SCHEDULE.at(i).min){

                    mNextAcqIndex = i;
                    break;

                }else if(date.minutes == m_CameraParam.schcap.ACQ_SCHEDULE.at(i).min){

                    if(date.seconds < m_CameraParam.schcap.ACQ_SCHEDULE.at(i).sec){

                        mNextAcqIndex = i;
                        break;

                    }
                }
            }
        }

        mNextAcq = m_CameraParam.schcap.ACQ_SCHEDULE.at(mNextAcqIndex);

    }

}

bool AcqThread::buildAcquisitionDirectory(string YYYYMMDD){
    LOG_DEBUG << "AcqThread::buildAcquisitionDirectory";

    string root = m_DataParam.DATA_PATH + m_StationParam.STATION_NAME + "_" + YYYYMMDD +"/";

    string subDir = "captures/";
    string finalPath = root + subDir;

    mOutputDataPath = finalPath;
    LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "CompleteDataPath : " << mOutputDataPath;

    path p(m_DataParam.DATA_PATH);
    path p1(root);
    path p2(root + subDir);

    // If DATA_PATH exists
    if(fs::exists(p)){

        // If DATA_PATH/STATI ON_YYYYMMDD/ exists
        if(fs::exists(p1)){

            // If DATA_PATH/STATION_YYYYMMDD/captures/ doesn't exists
            if(!fs::exists(p2)){

                // If fail to create DATA_PATH/STATION_YYYYMMDD/captures/
                if(!fs::create_directory(p2)){

                    LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create captures directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/captures/
                }else{

                    LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create captures directory : " << p2.string();
                   return true;

                }
            }

        // If DATA_PATH/STATION_YYYYMMDD/ doesn't exists
        }else{

            // If fail to create DATA_PATH/STATION_YYYYMMDD/
            if(!fs::create_directory(p1)){

               LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create STATION_YYYYMMDD directory : " << p1.string();
                return false;

            // If success to create DATA_PATH/STATION_YYYYMMDD/
            }else{

                LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create STATION_YYYYMMDD directory : " << p1.string();

                // If fail to create DATA_PATH/STATION_YYYYMMDD/stack/
                if(!fs::create_directory(p2)){

                    LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create captures directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/stack/
                }else{

                    LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create captures directory : " << p2.string();
                    return true;

                }
            }
        }

    // If DATA_PATH doesn't exists
    }else{

        // If fail to create DATA_PATH
        if(!fs::create_directory(p)){

          LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create DATA_PATH directory : " << p.string();
            return false;

        // If success to create DATA_PATH
        }else{

            LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create DATA_PATH directory : " << p.string();

            // If fail to create DATA_PATH/STATION_YYYYMMDD/
            if(!fs::create_directory(p1)){

                LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create STATION_YYYYMMDD directory : " << p1.string();
                return false;

            // If success to create DATA_PATH/STATION_YYYYMMDD/
            }else{

                LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create STATION_YYYYMMDD directory : " << p1.string();

                // If fail to create DATA_PATH/STATION_YYYYMMDD/captures/
                if(!fs::create_directory(p2)){

                    LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create captures directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/captures/
                }else{

                    LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create captures directory : " << p2.string();
                    return true;

                }
            }
        }
    }

    return true;
}

/// <summary>
/// Called to take regular and scheduled acquisition.
/// Single frame camera mode needed!
/// </summary>
/// 
/// <param name="imgNumber"></param>
/// <param name="imgExposure"></param>
/// <param name="imgGain"></param>
/// <param name="imgFormat"></param>
/// <param name="imgOutput"></param>
/// <param name="imgPrefix"></param>
void AcqThread::runImageCapture(EAcquisitionMode mode,int imgNumber, ImgFormat imgOutput, string imgPrefix) {
    LOG_DEBUG << "AcqThread::runImageCapture";

    if (mode == EAcquisitionMode::REGULAR)
        LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Running regular capture ";

    if (mode == EAcquisitionMode::SCHEDULED)
        LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Running scheduled capture ";

    // Stop stack process.
    LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "AcqThread::runImageCapture;" << "Stopping stack process";
    stopStackThread();
    // Stop detection process.

    LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "AcqThread::runImageCapture;" << "Stopping detection process";
    stopDetectionThread();

    // Reset framebuffer.
    resetFrameBuffer();
    
    for(int i = 0; i < imgNumber; i++) 
    {
        LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Prepare capture # " << i;
        // Configuration for single capture.
        shared_ptr<Frame> frame = make_shared<Frame>();

        ApplyCameraSettingsToFrame(frame);

        // Run single capture.
        LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Run single capture.";
        if(m_Device->runSingleCapture(frame)) {
            LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Single capture succeed !";
            LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "AcqThread::runImageCapture;" << "Single capture succeed !";
            saveImageCaptured(frame, i, imgOutput, imgPrefix);
        } else {
            LOG_ERROR << "AcqThread::runImageCapture;" << "Single capture failed !";
        }

    }

    //boost::this_thread::sleep(boost::posix_time::millisec(5));
 
    LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "Restarting camera in continuous mode...";

    if (!setCameraInContinousMode()) {
        throw runtime_error("Restart camera in continuous mode impossible");
    }

}

void AcqThread::saveImageCaptured(shared_ptr<Frame> frame, int imgNum, ImgFormat outputType, string imgPrefix) {
    LOG_DEBUG << "AcqThread::saveImageCaptured;\t\t"<< "AcqThread: SAVING IMAGE CAPTURED";

    if(frame->Image->data) {

        string  YYYYMMDD = TimeDate::getYYYYMMDD(frame->mDate);

        if(buildAcquisitionDirectory(YYYYMMDD)) {

            string fileName = imgPrefix + "_" + TimeDate::getYYYYMMDDThhmmss(frame->mDate) + "_UT-" + Conversion::intToString(imgNum);

            switch(outputType) {

                case JPEG :

                    {

                        switch(frame->mFormat) {

                            case CamPixFmt::MONO12 :

                                {

                                    cv::Mat temp;
                                    frame->Image->copyTo(temp);
                                    cv::Mat newMat = ImgProcessing::correctGammaOnMono12(temp, 2.2);
                                    cv::Mat newMat2 = Conversion::convertTo8UC1(newMat);
                                    SaveImg::saveJPEG(newMat2, mOutputDataPath + fileName);

                                }

                                break;

                            default :

                                {

                                    cv::Mat temp;
                                    frame->Image->copyTo(temp);
                                    cv::Mat newMat = ImgProcessing::correctGammaOnMono8(temp, 2.2);
                                    SaveImg::saveJPEG(newMat, mOutputDataPath + fileName);

                                }

                        }
                    }

                    break;

                case FITS :

                    {

                        Fits2D newFits(mOutputDataPath);
                        newFits.loadKeys(mfkp, m_StationParam);
                        newFits.kSATURATE = frame->mSaturatedValue;
                        newFits.kCAMERA = m_Device->getDeviceName();
                        newFits.kGAINDB = frame->mGain;
                        newFits.kEXPOSURE = frame->mExposure/1000000.0;
                        newFits.kONTIME = frame->mExposure/1000000.0;
                        newFits.kELAPTIME = frame->mExposure/1000000.0;
                        newFits.kDATEOBS = TimeDate::getIsoExtendedFormatDate(frame->mDate);

                        double  debObsInSeconds = frame->mDate.hours*3600 + frame->mDate.minutes*60 + frame->mDate.seconds;
                        double  julianDate      = TimeDate::gregorianToJulian(frame->mDate);
                        double  julianCentury   = TimeDate::julianCentury(julianDate);

                        newFits.kCRVAL1 = TimeDate::localSideralTime_2(julianCentury, frame->mDate.hours, frame->mDate.minutes, (int)frame->mDate.seconds, m_StationParam.SITELONG);
                        newFits.kCTYPE1 = "RA---ARC";
                        newFits.kCTYPE2 = "DEC--ARC";
                        newFits.kEQUINOX = 2000.0;

                        switch(frame->mFormat) {

                            case CamPixFmt::MONO12 :

                                {

                                    // Convert unsigned short type image in short type image.
                                    shared_ptr<cv::Mat> newMat = make_shared<cv::Mat>(frame->Image->rows, frame->Image->cols, CV_16SC1, cv::Scalar(0));

                                    // Set bzero and bscale for print unsigned short value in soft visualization.
                                    newFits.kBZERO = 32768;
                                    newFits.kBSCALE = 1;

                                    unsigned short *ptr = NULL;
                                    short *ptr2 = NULL;

                                    for(int i = 0; i < frame->Image->rows; i++){

                                        ptr = frame->Image->ptr<unsigned short>(i);
                                        ptr2 = newMat->ptr<short>(i);

                                        for(int j = 0; j < frame->Image->cols; j++){

                                            if(ptr[j] - 32768 > 32767){

                                                ptr2[j] = 32767;

                                            }else{

                                                ptr2[j] = ptr[j] - 32768;
                                            }
                                        }
                                    }

                                    // Create FITS image with BITPIX = SHORT_IMG (16-bits signed integers), pixel with TSHORT (signed short)
                                    if(newFits.writeFits(newMat, S16, fileName))
                                        LOG_DEBUG << "AcqThread::saveImageCaptured;\t\t" << "Fits saved in : " << mOutputDataPath << fileName;

                                }

                                break;

                            default :

                                {

                                   if(newFits.writeFits(frame->Image, UC8, fileName))
                                        LOG_DEBUG << "AcqThread::saveImageCaptured;\t\t" << "Fits saved in : " << mOutputDataPath << fileName;

                                }

                        }

                    }

                    break;

            }

        }
    }

}

int AcqThread::getNowInSeconds()
{
    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
    return getTimeInSeconds(time);
}

int AcqThread::getTimeInSeconds(boost::posix_time::ptime time)
{
    string date = to_iso_extended_string(time);
    vector<int> intDate = TimeDate::getIntVectorFromDateString(date);
    return intDate.at(3) * 3600 + intDate.at(4) * 60 + intDate.at(5);
}

bool AcqThread::computeSunTimes() {
    LOG_DEBUG << "AcqThread::computeSunTimes";
    
    int sunriseStartH = 0, sunriseStartM = 0, sunriseStopH = 0, sunriseStopM = 0,
        sunsetStartH = 0, sunsetStartM = 0, sunsetStopH = 0, sunsetStopM = 0;

    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
    string date = to_iso_extended_string(time);
    vector<int> intDate = TimeDate::getIntVectorFromDateString(date);

    string month = Conversion::intToString(intDate.at(1));
    if(month.size() == 1) month = "0" + month;
    string day = Conversion::intToString(intDate.at(2));
    if(day.size() == 1) day = "0" + day;
    
    mCurrentDate = Conversion::intToString(intDate.at(0)) + month + day;
    mCurrentTime = intDate.at(3) * 3600 + intDate.at(4) * 60 + intDate.at(5);

    LOG_DEBUG << "AcqThread::computeSunTimes;\t\t" << "LOCAL DATE      :  " << mCurrentDate;

    if(m_CameraParam.ephem.EPHEMERIS_ENABLED) {
        LOG_INFO << "COMPUTE SUN TIMES WITH EPHEMERIS";
        Ephemeris ephem1 = Ephemeris(mCurrentDate, m_CameraParam.ephem.SUN_HORIZON_1, m_StationParam.SITELONG, m_StationParam.SITELAT);

        if(!ephem1.computeEphemeris(sunriseStartH, sunriseStartM,sunsetStopH, sunsetStopM)) {

            return false;

        }

        Ephemeris ephem2 = Ephemeris(mCurrentDate, m_CameraParam.ephem.SUN_HORIZON_2, m_StationParam.SITELONG, m_StationParam.SITELAT );

        if(!ephem2.computeEphemeris(sunriseStopH, sunriseStopM,sunsetStartH, sunsetStartM)) {

            return false;

        }

    }else {
        LOG_INFO << "COMPUTE DEFAULT SUN TIMES";
        sunriseStartH = m_CameraParam.ephem.SUNRISE_TIME.at(0);
        sunriseStartM = m_CameraParam.ephem.SUNRISE_TIME.at(1);

        double intpart1 = 0;
        double fractpart1 = modf((double)m_CameraParam.ephem.SUNRISE_DURATION/3600.0 , &intpart1);

        if(intpart1!=0) {

            if(sunriseStartH + intpart1 < 24) {

                sunriseStopH = sunriseStartH + intpart1;


            }else {

                sunriseStopH = sunriseStartH + intpart1 - 24;

            }

        }else {

            sunriseStopH = sunriseStartH;

        }

        double intpart2 = 0;
        double fractpart2 = modf(fractpart1 * 60 , &intpart2);

        if(sunriseStartM + intpart2 < 60) {

            sunriseStopM = sunriseStartM + intpart2;

        }else {


            if(sunriseStopH + 1 < 24) {

                sunriseStopH += 1;

            }else {

                sunriseStopH = sunriseStopH + 1 - 24;

            }


            sunriseStopM = intpart2;

        }

        sunsetStartH = m_CameraParam.ephem.SUNSET_TIME.at(0);
        sunsetStartM = m_CameraParam.ephem.SUNSET_TIME.at(1);

        double intpart3 = 0;
        double fractpart3 = modf((double)m_CameraParam.ephem.SUNSET_DURATION/3600.0 , &intpart3);

        if(intpart3!=0) {

            if(sunsetStartH + intpart3 < 24) {

                sunsetStopH = sunsetStartH + intpart3;

            }else {

                sunsetStopH = sunsetStartH + intpart3 - 24;

            }

        }else {

            sunsetStopH = sunsetStartH;

        }

        double intpart4 = 0;
        double fractpart4 = modf(fractpart3 * 60 , &intpart4);

        if(sunsetStartM + intpart4 < 60) {

            sunsetStopM = sunsetStartM + intpart4;

        }else {


            if(sunsetStopH + 1 < 24) {

                sunsetStopH += 1;

            }else {

                sunsetStopH = sunsetStopH + 1 - 24;

            }

            sunsetStopM = intpart4;

        }

    }

    LOG_DEBUG << "AcqThread::computeSunTimes;\t\t" << "SUNRISE         :  " << sunriseStartH << "H" << sunriseStartM << " - " << sunriseStopH << "H" << sunriseStopM;
    LOG_DEBUG << "AcqThread::computeSunTimes;\t\t" << "SUNSET          :  " << sunsetStartH << "H" << sunsetStartM << " - " << sunsetStopH << "H" << sunsetStopM;

    mStartSunriseTime = sunriseStartH * 3600 + sunriseStartM * 60;
    mStopSunriseTime = sunriseStopH * 3600 + sunriseStopM * 60;
    mStartSunsetTime = sunsetStartH * 3600 + sunsetStartM * 60;
    mStopSunsetTime = sunsetStopH * 3600 + sunsetStopM * 60;

    return true;

}



// mCurrentTime is in seconds and starts from 0 to 24*3600=86400
// int is a wrong container even if on x64 4 bytes are used to make a int.
// better to use a specific datetime container eg. boost
// using minors because if is exactly the second of sunrise/sunset start it is not considered night
bool AcqThread::isNight(int currentTimeInSec)
{
    if ((currentTimeInSec < mStartSunriseTime) || (currentTimeInSec > mStopSunsetTime) ) 
        return true;

    return false;
}

bool AcqThread::isDay(int seconds)
{
    if ( seconds > mStopSunriseTime && seconds < mStartSunsetTime )
        return true;

    return false;
}

bool AcqThread::isNight()
{
    return isNight(mCurrentTime);
}

bool AcqThread::isDay()
{
    return isDay(mCurrentTime);
}

bool AcqThread::isSunset()
{
    return isSunset(mCurrentTime);
}

bool AcqThread::isSunrise()
{
    return isSunrise(mCurrentTime);
}

bool AcqThread::isSunrise(int seconds){
    if ( seconds >= mStartSunriseTime && seconds <= mStopSunriseTime ) 
        return true;
    return false;             
}

bool AcqThread::isSunset(int seconds){
    if ( seconds >= mStartSunsetTime  && seconds <= mStopSunsetTime )
        return true;

     return false;
}

TimeMode AcqThread::getTimeMode(int seconds)
{
    if (isNight(seconds)){
        return TimeMode::NIGHT;
    }

    if (isSunrise(seconds)){
        return TimeMode::SUNRISE;
    }
    if (isSunset(seconds)){
        return TimeMode::SUNSET;
    }

    if (isDay(seconds)){
        return TimeMode::DAY;
    }
    
    return TimeMode::NONE;
}



TimeMode AcqThread::getCurrentTimeMode()
{
    if (isNight()){
        return TimeMode::NIGHT;
    }

    if (isSunrise()){
        return TimeMode::SUNRISE;
    }
    if (isSunset()){
        return TimeMode::SUNSET;
    }

    if (isDay()){
        return TimeMode::DAY;
    }
    
    return TimeMode::NONE;
}

void AcqThread::ApplyCameraSettingsToFrame(shared_ptr<Frame> frame)
{
    if (!frame)
        throw runtime_error("Frame is null");

    LOG_DEBUG << "AcqThread::ApplyCameraSettingsToFrame";
    frame->mExposure = m_CurrentCameraSettings.Exposure;
    
    frame->mFormat = m_CurrentCameraSettings.PixelFormat;

    switch (frame->mFormat) {
    case CamPixFmt::MONO12:
        frame->mSaturatedValue = 4095;
        break;
    default:
        frame->mSaturatedValue = 255;
    }

    frame->mFps = m_CurrentCameraSettings.FPS;
    frame->mGain = m_CurrentCameraSettings.Gain;
    frame->mHeight = m_CurrentCameraSettings.SizeHeight;
    frame->mWidth = m_CurrentCameraSettings.SizeWidth; 
    frame->mStartX = m_CurrentCameraSettings.StartX;
    frame->mStartY = m_CurrentCameraSettings.StartY;

    LOG_INFO << "AcqThread::ApplyCameraSettingsToFrame;\t\t" << "Exposure time : " << frame->mExposure;
    LOG_INFO << "AcqThread::ApplyCameraSettingsToFrame;\t\t" << "Gain : " << frame->mGain;
    EParser<CamPixFmt> format;
    LOG_INFO << "AcqThread::ApplyCameraSettingsToFrame;\t\t" << "Format : " << format.getStringEnum(frame->mFormat);
    LOG_INFO << "AcqThread::ApplyCameraSettingsToFrame;\t\t" << "FPS : " << frame->mFps;
}

bool AcqThread::prepareAcquisitionOnDevice(EAcquisitionMode mode)
{
    LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice";

    // Get Sunrise start/stop, Sunset start/stop. ---
    computeSunTimes();

    if (mode == EAcquisitionMode::SCHEDULED)
    {
        m_CurrentCameraSettings.Gain = mNextAcq.gain;
        m_CurrentCameraSettings.Exposure = mNextAcq.exp;
        m_CurrentCameraSettings.FPS = 0;
        m_CurrentCameraSettings.PixelFormat = mNextAcq.fmt;
    }
    else {
        // CHECK SUNRISE AND SUNSET TIMES.
        if (isNight())
        {
            LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "*************************** SET NIGHT CAMERA SETTINGS";
            LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "AUTO EXPOSURE   :  NO";
            LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "DAYTIME         :  NO";

            //taking parameters from current configuration object
            switch (mode) {
            case EAcquisitionMode::CONTINUOUS: {
                m_CurrentCameraSettings.Gain = m_CameraParam.ACQ_NIGHT_GAIN;
                m_CurrentCameraSettings.Exposure = m_CameraParam.ACQ_NIGHT_EXPOSURE;
                m_CurrentCameraSettings.FPS = m_CameraParam.ACQ_FPS;
                m_CurrentCameraSettings.PixelFormat = m_CameraParam.ACQ_FORMAT;
                break;
            }
            case EAcquisitionMode::REGULAR: {
                m_CurrentCameraSettings.Gain = m_CameraParam.regcap.ACQ_REGULAR_CFG.gain;
                m_CurrentCameraSettings.Exposure = m_CameraParam.regcap.ACQ_REGULAR_CFG.exp;
                m_CurrentCameraSettings.FPS = 0;
                m_CurrentCameraSettings.PixelFormat = m_CameraParam.regcap.ACQ_REGULAR_CFG.fmt;
                break;
            }
            };
        }
        else
        {
            if (isDay()) {
                LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "*************************** SET DAY CAMERA SETTINGS";
                LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "AUTO EXPOSURE   :  NO";
                LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "DAYTIME         :  YES";

                switch (mode) {
                case EAcquisitionMode::CONTINUOUS: {
                    m_CurrentCameraSettings.Gain = m_CameraParam.ACQ_DAY_GAIN;
                    m_CurrentCameraSettings.Exposure = m_CameraParam.ACQ_DAY_EXPOSURE;
                    m_CurrentCameraSettings.FPS = m_CameraParam.ACQ_FPS;
                    m_CurrentCameraSettings.PixelFormat = m_CameraParam.ACQ_FORMAT;
                    break;
                }
                case EAcquisitionMode::REGULAR: {
                    m_CurrentCameraSettings.Gain = m_CameraParam.regcap.ACQ_REGULAR_CFG.gain;
                    m_CurrentCameraSettings.Exposure = m_CameraParam.ACQ_DAY_EXPOSURE;
                    m_CurrentCameraSettings.FPS = 0;
                    m_CurrentCameraSettings.PixelFormat = m_CameraParam.regcap.ACQ_REGULAR_CFG.fmt;
                    break;
                }
                };
            }
            else {
                LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "*************************** SET SUNRISE/SUNSET CAMERA SETTINGS";
                LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "AUTO EXPOSURE   :  NO";
                LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "DAYTIME         :  YES";

                //SUNRISE AND SUNSET
                switch (mode) {
                case EAcquisitionMode::CONTINUOUS: {
                    m_CurrentCameraSettings.Gain = m_CameraParam.ACQ_DAY_GAIN;
                    m_CurrentCameraSettings.Exposure = m_CameraParam.ACQ_DAY_EXPOSURE;
                    m_CurrentCameraSettings.FPS = m_CameraParam.ACQ_FPS;
                    m_CurrentCameraSettings.PixelFormat = m_CameraParam.ACQ_FORMAT;
                    break;
                }
                case EAcquisitionMode::REGULAR: {
                    m_CurrentCameraSettings.Gain = m_CameraParam.regcap.ACQ_REGULAR_CFG.gain;
                    m_CurrentCameraSettings.Exposure = m_CameraParam.ACQ_NIGHT_EXPOSURE;
                    m_CurrentCameraSettings.FPS = 0;
                    m_CurrentCameraSettings.PixelFormat = m_CameraParam.regcap.ACQ_REGULAR_CFG.fmt;
                    break;
                }
                };
            }
        }
    }

    LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "EXPOSURE TIME   :  " << m_CurrentCameraSettings.Exposure;
    LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "GAIN            :  " << m_CurrentCameraSettings.Gain;
    LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "FPS             :  " << m_CurrentCameraSettings.FPS;
    
    //Set configuration to device
    m_Device->CameraSetting = m_CurrentCameraSettings;

    // INIT CAMERA.
    if(!m_Device->initializeCamera())
        return false;

    // START CAMERA.
    if(!m_Device->startCamera(mode))
        return false;

    return true;
}

/// <summary>
/// 
/// </summary>
void AcqThread::metrics_ThreadLoop()
{
    //calculate floating average for FPS circular buffer the same size as the frame buffer

    //if runned a scheduled or regular acquisition, jump this fps value. is not valid.
    if ( m_RunnedScheduledAcquisition || m_RunnedRegularAcquisition )
    {
        m_LastMetricTimestamp = m_CurrentThreadLoopTime;
        return;
    }

    m_FPS_Sum += m_CurrentFPS;

    if (m_CurrentFPS < 30)
        LOG_WARNING << "FPS is below 30 " << m_CurrentFPS;

    if (m_FPS_metrics.size() >= m_CircularBufferSize)
        m_FPS_Sum -= m_FPS_metrics.front();

    m_FPS_metrics.push_back(m_CurrentFPS);

    //UPDATE NODE EXPORTER METRICS
    boost::posix_time::time_duration metrics_time_elapsed = m_CurrentThreadLoopTime - m_LastMetricTimestamp;
    int64_t secTime = metrics_time_elapsed.total_seconds();

    if (secTime > METRICS_ACQUISITION_TIME)
    {

        double fps_floating_average = m_FPS_Sum / ( m_FPS_metrics.size() )  ;
        double temperature = m_Device->getTemperature();
        NodeExporterMetrics::GetInstance(m_StationParam.STATION_NAME).UpdateMetrics(fps_floating_average, tacq_metric, temperature, &nextSunrise, &nextSunset);
        NodeExporterMetrics::GetInstance(m_StationParam.STATION_NAME).WriteMetrics();

        m_LastMetricTimestamp = m_CurrentThreadLoopTime;
    }
}

void AcqThread::scheduledAcquisition_ThreadLoop()
{
    // ACQUISITION AT SCHEDULED TIME IS ENABLED.
    if (m_CameraParam.schcap.ACQ_SCHEDULE.size() != 0 && m_CameraParam.schcap.ACQ_SCHEDULE_ENABLED)
    {
        unsigned long next = (mNextAcq.hours * 3600 + mNextAcq.min * 60 + mNextAcq.sec) - (m_CurrentTimeDate.hours * 3600 + m_CurrentTimeDate.minutes * 60 + m_CurrentTimeDate.seconds);

        if (next < 0) {
            next = 24l * 3600l - (m_CurrentTimeDate.hours * 3600 + m_CurrentTimeDate.minutes * 60 + m_CurrentTimeDate.seconds) + (mNextAcq.hours * 3600 + mNextAcq.min * 60 + mNextAcq.sec);
            if (LOG_SPAM_FRAME_STATUS)
                LOG_DEBUG << "AcqThread::operator();" << "next : " << next;
        }

        vector<int>tsch = TimeDate::HdecimalToHMS(next / 3600.0);
        if (LOG_SPAM_FRAME_STATUS)
            LOG_DEBUG << "AcqThread::operator();" << "NEXT SCHCAP : " << tsch.at(0) << "h" << tsch.at(1) << "m" << tsch.at(2) << "s" << endl;

        // It's time to run scheduled acquisition.
        if (mNextAcq.hours == m_CurrentTimeDate.hours &&
            mNextAcq.min == m_CurrentTimeDate.minutes &&
            (int)m_CurrentTimeDate.seconds == mNextAcq.sec) {
            runScheduledAcquisition(m_CurrentTimeDate);
        }
        else {

            // The current time has elapsed.
            if (m_CurrentTimeDate.hours > mNextAcq.hours) {

                selectNextAcquisitionSchedule(m_CurrentTimeDate);

            }
            else if (m_CurrentTimeDate.hours == mNextAcq.hours) {

                if (m_CurrentTimeDate.minutes > mNextAcq.min) {

                    selectNextAcquisitionSchedule(m_CurrentTimeDate);

                }
                else if (m_CurrentTimeDate.minutes == mNextAcq.min) {

                    if (m_CurrentTimeDate.seconds > mNextAcq.sec) {

                        selectNextAcquisitionSchedule(m_CurrentTimeDate);
                    }

                }

            }

        }


        if (m_CurrentAcquisitionMode != EAcquisitionMode::CONTINUOUS)
        {
            LOG_INFO << "AcqThread::operator();" << "Set camera in continuous mode";
            setCameraInContinousMode();
        }
    }
}

void AcqThread::regularAcquisition_ThreadLoop()
{
    // ACQUISITION AT REGULAR TIME INTERVAL IS ENABLED
    if (m_CameraParam.regcap.ACQ_REGULAR_ENABLED)
    {
        boost::posix_time::time_duration regular_acq_time_elapsed = m_CurrentThreadLoopTime - m_LastRegularAcquisitionTimestamp;
        int64_t secTime = regular_acq_time_elapsed.total_seconds();

        if (LOG_SPAM_FRAME_STATUS)
            LOG_DEBUG << "AcqThread::operator();" << "operator();" << "NEXT REGCAP : " << (int)(m_CameraParam.regcap.ACQ_REGULAR_CFG.interval - secTime) << "s" << endl;

        // Check it's time to run a regular capture.
        if (secTime >= m_CameraParam.regcap.ACQ_REGULAR_CFG.interval)
        {
            LOG_DEBUG << "AcqThread::operator();" << "operator();" << "It's time to check if need to run a regular capture.";
            if ((m_CurrentTimeMode == m_CameraParam.regcap.ACQ_REGULAR_MODE || m_CameraParam.regcap.ACQ_REGULAR_MODE == DAYNIGHT)) {
                LOG_INFO << "AcqThread::operator();" << "============= Run regular acquisition because it is " << m_CurrentTimeMode << " and ACQ_REGULAR_MODE=" << m_CameraParam.regcap.ACQ_REGULAR_MODE << " =============";
                runRegularAcquisition();

                // Reset reference time in case a long exposure has been done.
                m_LastRegularAcquisitionTimestamp = m_CurrentThreadLoopTime;
            }

            if (m_CurrentAcquisitionMode != EAcquisitionMode::CONTINUOUS)
            {
                LOG_INFO << "AcqThread::operator();" << "Set camera in continuous mode";
                setCameraInContinousMode();
            }
        }
    }
}

void AcqThread::continuousAcquisition_ThreadLoop()
{
    InputDeviceType input_device_type = m_Device->getDeviceType();

    m_CurrentFrame = make_shared<Frame>(); // Container for the grabbed image. added to the frame vector. need to be new object
    m_CurrentFrame->mDate = m_CurrentTimeDate;

    ApplyCameraSettingsToFrame(m_CurrentFrame);

    // GRAB A CONTINUOUS FRAME
    if (m_Device->runContinuousCapture(m_CurrentFrame))
    {
        m_CurrentFrame->mFrameNumber = mFrameNumber;

        if (LOG_SPAM_FRAME_STATUS)
            LOG_INFO << "AcqThread::operator();" << "============= FRAME " << m_CurrentFrame->mFrameNumber << " ============= ";


        switch (input_device_type) {
        // IF DEVICE IS FRAME OR VIDEO...
        case InputDeviceType::SINGLE_FITS_FRAME:
        case InputDeviceType::VIDEO:
            continuousAcquisition_VIDEO();
            break;

        //IF DEVICE IS A CAMERA... EXPOSURE CONTROL MANAGEMENT
        case InputDeviceType::CAMERA:
            continuousAcquisition_CAMERA();
            break;

        }

        //PUSH THE NEW FRAME IN THE FRAMEBUFFER AFTER EVENTUALLY THREADS STOP
        boost::mutex::scoped_lock lock(*frameBuffer_mutex);
        frameBuffer.push_back(m_CurrentFrame);
        lock.unlock();


        // Notify detection thread for new frame.
        if (m_DetThread_IsRunning)
            startDetectionThread();

        // Notify stack thread for new frame.
        if (m_StackThread_IsRunning)
            startStackThread();

        
        mFrameNumber++;

        //LOG
        if (LOG_SPAM_FRAME_STATUS)
            LOG_INFO << "AcqThread::operator();" << " [ TIME ACQ ] : " << tacq_metric << " ms   ~cFPS(" << m_CurrentFPS << ")" << endl;

        
        
    }
}

void AcqThread::continuousAcquisition_VIDEO()
{

    // Slow down the time in order to give more time to the detection process.
    int twait = DEFAULT_VIDEO_WAIT_BETWEEN_FRAMES;
    if (m_VideoParam.INPUT_TIME_INTERVAL == 0 && m_FramesParam.INPUT_TIME_INTERVAL > 0)
        twait = m_FramesParam.INPUT_TIME_INTERVAL;
    else if (m_VideoParam.INPUT_TIME_INTERVAL > 0 && m_FramesParam.INPUT_TIME_INTERVAL == 0)
        twait = m_VideoParam.INPUT_TIME_INTERVAL;

    boost::this_thread::sleep(boost::posix_time::millisec(twait * 1000));

}

void AcqThread::continuousAcquisition_CAMERA()
{
    //IF EXPOSURE CONTROL IS ENABLED
    if (m_DetectionParam.ACQ_AUTOEXPOSURE_ENABLED)
    {
        if ((isSunrise(m_CurrentTimeInSeconds) || isSunset(m_CurrentTimeInSeconds))) {

            exposureControlActive = true;

        }
        else
        {
            exposureControlActive = false;
            exposureControlStatus = false;
        }

        if (exposureControlStatus) {
            // Exposure control is active, the new frame can't be shared with others threads.
            if (!cleanStatus) {
                // If stack process exists.
                stopStackThread();

                // If detection process exists
                stopDetectionThread();

                // Reset framebuffer.
                resetFrameBuffer();
            }
        }


        // Adjust exposure time.
        if (pExpCtrl != NULL && exposureControlActive)
            exposureControlStatus = pExpCtrl->controlExposureTime(m_Device, m_CurrentFrame->Image, m_CurrentFrame->mDate, m_DetectionParam.MASK, m_CameraParam.ACQ_FPS);

    }


    //IF EXPOSURE CONTROL IS DISABLED SETUP DET AND STACK THREADS
    if (!m_DetectionParam.ACQ_AUTOEXPOSURE_ENABLED && !exposureControlStatus)
    {
        // Check TimeMode changing.
        // Notify detection thread.
        if (previousTimeMode != m_CurrentTimeMode && m_DetectionParam.DET_MODE != DAYNIGHT) {
            LOG_INFO << "AcqThread::operator();" << "TimeMode has changed and Detection thread need to be stopped because it is " << time_mode_parser.getStringEnum(m_CurrentTimeMode) << " and DET_MODE =" << time_mode_parser.getStringEnum(m_StackParam.STACK_MODE);
            //stopDetectionThread();
            m_DetThread_IsRunning = false;
        }
        else if (m_DetectionParam.DET_MODE == m_CurrentTimeMode || m_DetectionParam.DET_MODE == DAYNIGHT) {
            if (!m_DetThread_IsRunning)
            {
                LOG_INFO << "AcqThread::operator();" << "TimeMode has changed and Detection thread need to be started because it is " << time_mode_parser.getStringEnum(m_CurrentTimeMode) << " and STACK_MODE=" << time_mode_parser.getStringEnum(m_StackParam.STACK_MODE);
                //startDetectionThread();
                m_DetThread_IsRunning = true;
            }
        }


        // Notify stack thread.
        if (previousTimeMode != m_CurrentTimeMode && m_StackParam.STACK_MODE != DAYNIGHT) {
            if (m_StackThread_IsRunning)
            {
                LOG_INFO << "AcqThread::operator();" << "TimeMode has changed and Stack thread need to be stopped because it is " << time_mode_parser.getStringEnum(m_CurrentTimeMode) << " and STACK_MODE=" << time_mode_parser.getStringEnum(m_StackParam.STACK_MODE);
                //stopStackThread();
                m_StackThread_IsRunning = false;
            }
        }
        else if (m_StackParam.STACK_MODE == m_CurrentTimeMode || m_StackParam.STACK_MODE == DAYNIGHT) {
            if (!m_StackThread_IsRunning) {
                LOG_INFO << "AcqThread::operator();" << "TimeMode has changed and Stack thread need to be started because it is " << time_mode_parser.getStringEnum(m_CurrentTimeMode) << " and STACK_MODE=" << time_mode_parser.getStringEnum(m_StackParam.STACK_MODE);
                //startStackThread();
                m_StackThread_IsRunning = true;
            }
        }
    }
}

