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

#include "Logger.h"

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
#include <boost/thread/future.hpp>

namespace fs = boost::filesystem;

using namespace std;
using namespace freeture;


// *************************** CTOR DTOR
AcqThread::AcqThread(
    boost::circular_buffer<shared_ptr<Frame>>& fb,
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
    LOG_DEBUG << "AcqThread::AcqThread" << endl;
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


/// **************************** THREADS MANAGEMENT
void AcqThread::stopThread(){
    LOG_DEBUG << "AcqThread::stopThread" << endl;

    mMustStopMutex.lock();
    mMustStop = true;
    mMustStopMutex.unlock();

    if(mThread != NULL)
        while(mThread->timed_join(boost::posix_time::seconds(2)) == false)
            mThread->interrupt();

}

bool AcqThread::startThread()
{
    LOG_DEBUG << "AcqThread::startThread" << endl;
    
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
        if (LOG_SPAM_FRAME_STATUS)
            LOG_DEBUG << "AcqThread::startDetectionThread; DETECTION THREAD START SIGNAL" << endl;
        boost::mutex::scoped_lock lock2(*detSignal_mutex);
        *detSignal = true;
        detSignal_condition->notify_one();
        lock2.unlock();
    }
}

void AcqThread::stopDetectionThread()
{
    if ( pDetection != nullptr ) {
        LOG_DEBUG << "AcqThread::stopDetectionThread; DETECTION THREAD STOP SIGNAL" << endl;

        boost::mutex::scoped_lock lock(*detSignal_mutex);
        *detSignal = false;
        lock.unlock();
        
        // Force interruption.
        LOG_DEBUG << "AcqThread::stopDetectionThread;" << "Sending interruption signal to detection process... " << endl;
        pDetection->interruptThread();
    }
}

void AcqThread::startStackThread()
{
    if (pStack != nullptr)
    {
        if (LOG_SPAM_FRAME_STATUS)
            LOG_DEBUG << "AcqThread::startStackThread; STACK THREAD START SIGNAL" << endl;

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
        LOG_DEBUG << "AcqThread::stopStackThread; STACK THREAD STOP SIGNAL" << endl;

        boost::mutex::scoped_lock lock(*stackSignal_mutex);
        *stackSignal = false;
        lock.unlock();

        // Force interruption.
        LOG_DEBUG <<"AcqThread::stopStackThread;" << "Send interruption signal to stack " << endl;
        pStack->interruptThread();
    }
}

void AcqThread::operator()()
{
    // Obtain the attribute_value which holds the thread ID
    m_ThreadID = std::this_thread::get_id();

    Logger::Get()->setLogThread(LogThread::ACQUISITION_THREAD, m_ThreadID, true);

    bool stop = false;


    LOG_INFO << "AcqThread::operator();" << "==============================================" << endl;
    LOG_INFO << "AcqThread::operator();" << "========== START ACQUISITION THREAD ==========" << endl;
    LOG_INFO << "AcqThread::operator();" << "==============================================" << endl;

    try {
        
   
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
       
        LOG_INFO << "CURRENT TIME MODE: " << time_mode_parser.getStringEnum(previousTimeMode) << endl;
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
            setCurrentTimeAndDate(m_CurrentThreadLoopTime);
            setCurrentTimeDateAndSeconds(m_CurrentThreadLoopTime);

            if (!setCameraInContinousMode()) {
                LOG_ERROR << "AcqThread::operator();" << "Cannot set continuous camera mode" << endl;
                break;
            }

            selectFirstScheduledAcquisition();

            do
            {
                //set current microseconds time clock
                m_CurrentThreadLoopTime = boost::posix_time::microsec_clock::universal_time();

                //init metrics
                tacq_metric = (double)cv::getTickCount();

                // ********************** ACQUISITION LOOP *************************** NEED TO BE RUNNED AS MUCH AS POSSIBLE NEAR 30Hz
                //  RUN STACK EVERY 60s     (1440 images)
                //  RUN CAPTURES EVERY 10m  (144 images)
                //  OR RUN SCHEDULED CAPTURES
                //
                // WRITE CODE TO BE NOT CLEARED EVERY TIME OF THE DAY... USELESS CARBON 
                nextSunset.clear();
                nextSunrise.clear();

                // set current times
                setCurrentTimeAndDate(m_CurrentThreadLoopTime);
                setCurrentTimeDateAndSeconds(m_CurrentThreadLoopTime);

                // Detect day or night.
                m_CurrentTimeMode       = getTimeMode(m_CurrentTimeInSeconds);

                if (m_CurrentTimeMode != previousTimeMode) {
                    switch (m_CurrentTimeMode) {
                    case TimeMode::DAY:
                        LOG_INFO << "AcqThread::operator();" << "Running on DAY" << endl;
                        break;
                    case TimeMode::NIGHT:
                        LOG_INFO << "AcqThread::operator();" << "Running on NIGHT" << endl;
                        break;
                    case TimeMode::SUNRISE:
                        LOG_INFO << "AcqThread::operator();" << "Running on SUNRISE" << endl;
                        break;
                    case TimeMode::SUNSET:
                        LOG_INFO << "AcqThread::operator();" << "Running on SUNSET" << endl;
                        break;
                    }
                }

                //CALCULATE SUN EPHEMERYS on data change
                // If the date has changed, sun ephemeris must be updated.
                if (m_PreviousThreadLoopTime.date() != m_CurrentThreadLoopTime.date())
                {
                    LOG_INFO << "AcqThread::operator();" << "Date has changed. Former Date is " << m_PreviousThreadLoopTime.date() << ". New Date is " << m_CurrentThreadLoopTime.date() << "." << endl;

                    computeSunTimes();

                    //jump to next loop iteration with new dates.
                    continue;
                }
               
                //check if device is connected.
                if (!m_Device->isConnected()) {
                    LOG_ERROR << "DEVICE IS NOT CONNECTED ANYMORE" << endl;
                    if (!m_Device->connect()) {
                        LOG_ERROR << "CONNECTION FAILED WAITING 1s" << endl;
                        boost::this_thread::sleep(boost::posix_time::millisec(1000));
                        m_ConnectionLost = true;
                        stopDetectionThread();
                        stopStackThread();

                        continue;
                    }
                }
                else
                {
                    m_ConnectionLost = false;
                }

                //run continuous acquisition (grab a new frame)
                continuousAcquisition_ThreadLoop();

                // Print time before sunrise.
                if (!isSunrise(m_CurrentTimeInSeconds)) {
                    nextSunrise.clear();
                    if (m_CurrentTimeInSeconds < mStartSunriseTime)
                        nextSunrise = TimeDate::HdecimalToHMS((mStartSunriseTime - m_CurrentTimeInSeconds) / 3600.0);
                    if (m_CurrentTimeInSeconds > mStopSunsetTime)
                        nextSunrise = TimeDate::HdecimalToHMS(((24 * 3600 - m_CurrentTimeInSeconds) + mStartSunriseTime) / 3600.0);
                    if (LOG_SPAM_FRAME_STATUS)
                        if (nextSunrise.size() > 0)
                            LOG_DEBUG << "AcqThread::operator();" << "NEXT SUNRISE : " << nextSunrise.at(0) << "h" << nextSunrise.at(1) << "m" << nextSunrise.at(2) << "s" << endl;
                }

                // Print time before sunset.
                if (!isSunset())
                {
                    nextSunset.clear();
                    nextSunset = TimeDate::HdecimalToHMS((mStartSunsetTime - m_CurrentTimeInSeconds) / 3600.0);
                    if (LOG_SPAM_FRAME_STATUS)
                        if (nextSunset.size() > 0)
                            LOG_DEBUG << "AcqThread::operator();" << "NEXT SUNSET : " << nextSunset.at(0) << "h" << nextSunset.at(1) << "m" << nextSunset.at(2) << "s" << endl;
                }


                //CALCULATE METRICS
                tacq_metric = ((static_cast<double>(cv::getTickCount()) - tacq_metric) / cv::getTickFrequency()) * 1000;
                m_CurrentFPS = (1.0 / (tacq_metric / 1000.0));

                metrics_ThreadLoop();


                //check if need to run regula acquisition
                regularAcquisition_ThreadLoop();

                //check if need to run scheduled acquisition
                scheduledAcquisition_ThreadLoop();

                //reset continuous mode if necessary - outside metrics
                resetContinuousMode();
             

                //reset mutex
                m_RunnedScheduledAcquisition = false;
                m_RunnedRegularAcquisition = false;

                //CHECK THREAD STOP
                mMustStopMutex.lock();
                stop = mMustStop;
                mMustStopMutex.unlock();

                //set previous timestamp
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
        LOG_WARNING << "AcqThread::operator();" << "AcqThread ended." << endl;
    }
    catch (exception& e) {
        LOG_ERROR << "AcqThread::operator();" << "An exception occured : " << e.what() << endl;
    }
    catch (const char* msg) {
        LOG_ERROR << "AcqThread::operator();" << msg << endl;
    }

    m_Device->stopCamera();

    mThreadTerminated = true;

    LOG_INFO << "AcqThread::operator();" << "Acquisition Thread TERMINATED" << endl;
}








// ****************** CAMERA MANAGEMENT

/// <summary>
/// Empty the frame buffer, used before taking regular or scheduled capture.
/// </summary>
void AcqThread::resetFrameBuffer()
{
    LOG_DEBUG << "AcqThread::resetFrameBuffer;" << "Cleaning frameBuffer..." << endl;
    boost::mutex::scoped_lock lock(*frameBuffer_mutex);

    frameBuffer.clear();

    mFrameNumber = 0;

    lock.unlock();

    cleanStatus = true;
}

/// <summary>
/// Set device in order to take a single frame
/// </summary>
/// <param name="mode"></param>
/// <returns></returns>
bool AcqThread::setCameraSingleFrameMode(EAcquisitionMode mode)
{

    LOG_DEBUG << "AcqThread::setCameraSingleMode" << endl;
    if (m_Device->isStreaming())
        m_Device->stopCamera();

    m_Device->Setup(m_CameraParam, m_FramesParam, m_VideoParam);

    LOG_DEBUG << "AcqThread::setCameraSingleMode;" << "Prepare single-frame acquisition." << endl;

    if (!prepareAcquisitionOnDevice(mode)) {
        LOG_DEBUG << "FAIL ON PREPARE AQ DEVICE" << endl;
        return false;
    }

    m_CurrentAcquisitionMode = mode;

    return true;
}

/// <summary>
/// Set device in order to take frames in continuous mode
/// </summary>
/// <returns></returns>
bool AcqThread::setCameraInContinousMode()
{
    LOG_DEBUG << "AcqThread::setCameraInContinousMode" << endl;

    if (m_Device->isStreaming())
        m_Device->stopCamera();

    m_Device->Setup(m_CameraParam, m_FramesParam, m_VideoParam);

    if (!prepareAcquisitionOnDevice(EAcquisitionMode::CONTINUOUS)) {
        LOG_DEBUG << "FAIL ON PREPARE AQ DEVICE" << endl;
        return false;
    }

    m_CurrentAcquisitionMode = EAcquisitionMode::CONTINUOUS;

    return true;
}

/// <summary>
/// Save camera settings to frame
/// </summary>
/// <param name="frame"></param>
void AcqThread::ApplyCameraSettingsToFrame(shared_ptr<Frame> frame)
{
    if (!frame)
        throw runtime_error("Frame is null");

    if (LOG_SPAM_FRAME_STATUS)
        LOG_DEBUG << "AcqThread::ApplyCameraSettingsToFrame" << endl;

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

    if (LOG_SPAM_FRAME_STATUS)
    {
        LOG_INFO << "AcqThread::ApplyCameraSettingsToFrame;\t\t" << "Exposure time : " << frame->mExposure << endl;
        LOG_INFO << "AcqThread::ApplyCameraSettingsToFrame;\t\t" << "Gain : " << frame->mGain << endl;
        EParser<CamPixFmt> format;
        LOG_INFO << "AcqThread::ApplyCameraSettingsToFrame;\t\t" << "Format : " << format.getStringEnum(frame->mFormat) << endl;
        LOG_INFO << "AcqThread::ApplyCameraSettingsToFrame;\t\t" << "FPS : " << frame->mFps << endl;
    }
}

/// <summary>
/// Configure camera settings, initializeCamera and start acquisition
/// </summary>
/// <param name="mode"></param>
/// <returns></returns>
bool AcqThread::prepareAcquisitionOnDevice(EAcquisitionMode mode)
{
    LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice" << endl;

    // Get Sunrise start/stop, Sunset start/stop. ---
    computeSunTimes();

    if (mode == EAcquisitionMode::SCHEDULED)
    {
        LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "*************************** SET SCHEDULED CAMERA SETTINGS" << endl;
        LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "AUTO EXPOSURE   :  NO" << endl;

        m_CurrentCameraSettings.Gain = mNextAcq.gain;
        m_CurrentCameraSettings.Exposure = mNextAcq.exp;
        m_CurrentCameraSettings.FPS = 0;
        m_CurrentCameraSettings.PixelFormat = mNextAcq.fmt;
    }
    else {
        // CHECK SUNRISE AND SUNSET TIMES.
        if (isNight())
        {
            LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "*************************** SET NIGHT CAMERA SETTINGS" << endl;
            LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "AUTO EXPOSURE   :  NO" << endl;
            LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "DAYTIME         :  NO" << endl;

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
                LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "*************************** SET DAY CAMERA SETTINGS" << endl;
                LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "AUTO EXPOSURE   :  NO" << endl;
                LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "DAYTIME         :  YES" << endl;

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
                LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "*************************** SET SUNRISE/SUNSET CAMERA SETTINGS" << endl;
                LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "AUTO EXPOSURE   :  NO" << endl;
                LOG_INFO << "AcqThread::prepareAcquisitionOnDevice  ;\t\t" << "DAYTIME         :  YES" << endl;

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

    LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "EXPOSURE TIME   :  " << m_CurrentCameraSettings.Exposure << endl;
    LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "GAIN            :  " << m_CurrentCameraSettings.Gain << endl;
    LOG_INFO << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "FPS             :  " << m_CurrentCameraSettings.FPS << endl;

    //Set configuration to device
    m_Device->CameraSetting = m_CurrentCameraSettings;

    // INIT CAMERA.
    LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "Device->initializeCamera" << endl;

    if (!m_Device->initializeCamera())
        return false;

    // START CAMERA.
    LOG_DEBUG << "AcqThread::prepareAcquisitionOnDevice;\t\t" << "Device->startCamera" << endl;
    if (!m_Device->startCamera(mode))
        return false;

    return true;
}

/// <summary>
/// Grab a frame and route the device frame implementation management.
/// Push the new frame into framebuffer
/// Notify detection and stack thread for a new frame
/// increase frame number
/// </summary>
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
            LOG_INFO << "AcqThread::operator();" << "============= FRAME " << m_CurrentFrame->mFrameNumber << " ============= " << endl;


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

/// <summary>
/// Manage AUTOEXPOSURE feature
/// Manage DAY NIGHT Detection and stack switches
/// </summary>
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
            LOG_INFO << "AcqThread::operator();" << "TimeMode has changed and Detection thread need to be stopped because it is " << time_mode_parser.getStringEnum(m_CurrentTimeMode) << " and DET_MODE =" << time_mode_parser.getStringEnum(m_StackParam.STACK_MODE) << endl;
            //stopDetectionThread();
            m_DetThread_IsRunning = false;
        }
        else if (m_DetectionParam.DET_MODE == m_CurrentTimeMode || m_DetectionParam.DET_MODE == DAYNIGHT) {
            if (!m_DetThread_IsRunning)
            {
                LOG_INFO << "AcqThread::operator();" << "TimeMode has changed and Detection thread need to be started because it is " << time_mode_parser.getStringEnum(m_CurrentTimeMode) << " and STACK_MODE=" << time_mode_parser.getStringEnum(m_StackParam.STACK_MODE) << endl;
                //startDetectionThread();
                m_DetThread_IsRunning = true;
            }
        }

        // Notify stack thread.
        if (previousTimeMode != m_CurrentTimeMode && m_StackParam.STACK_MODE != DAYNIGHT) {
            if (m_StackThread_IsRunning)
            {
                LOG_INFO << "AcqThread::operator();" << "TimeMode has changed and Stack thread need to be stopped because it is " << time_mode_parser.getStringEnum(m_CurrentTimeMode) << " and STACK_MODE=" << time_mode_parser.getStringEnum(m_StackParam.STACK_MODE) << endl;
                //stopStackThread();
                m_StackThread_IsRunning = false;
            }
        }
        else if (m_StackParam.STACK_MODE == m_CurrentTimeMode || m_StackParam.STACK_MODE == DAYNIGHT) {
            if (!m_StackThread_IsRunning) {
                LOG_INFO << "AcqThread::operator();" << "TimeMode has changed and Stack thread need to be started because it is " << time_mode_parser.getStringEnum(m_CurrentTimeMode) << " and STACK_MODE=" << time_mode_parser.getStringEnum(m_StackParam.STACK_MODE) << endl;
                //startStackThread();
                m_StackThread_IsRunning = true;
            }
        }
    }
}

/// <summary>
/// Called to take regular and scheduled acquisition.
/// Single frame camera mode needed!
/// 
/// - Stops detection and stack thread
/// - reset frame buffer
/// - create new frame applying camera settings
/// - run a single frame capture
/// - save the image
/// - reset continuous mode
/// </summary>
/// 
/// <param name="imgNumber"></param>
/// <param name="imgExposure"></param>
/// <param name="imgGain"></param>
/// <param name="imgFormat"></param>
/// <param name="imgOutput"></param>
/// <param name="imgPrefix"></param>
void AcqThread::runImageCapture(EAcquisitionMode mode,int imgNumber, ImgFormat imgOutput, string imgPrefix) {
    LOG_DEBUG << "AcqThread::runImageCapture" << endl;
    LOG_DEBUG << "PERFORMANCE METRICS START runImageCapture";
    if (mode == EAcquisitionMode::REGULAR)
        LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Running regular capture " << endl;

    if (mode == EAcquisitionMode::SCHEDULED)
        LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Running scheduled capture " << endl;

    // Stop stack process.
    LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "AcqThread::runImageCapture;" << "Stopping stack process" << endl;
    stopStackThread();
    // Stop detection process.

    LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "AcqThread::runImageCapture;" << "Stopping detection process" << endl;
    stopDetectionThread();

    // Reset framebuffer.
    LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "AcqThread::runImageCapture;" << "Reset frame buffer" << endl;
    resetFrameBuffer();
    
    for(int i = 0; i < imgNumber; i++) 
    {
        LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Prepare capture # " << i << endl;
        // Configuration for single capture.
        shared_ptr<Frame> frame = make_shared<Frame>();

        ApplyCameraSettingsToFrame(frame);

        // Run single capture.
        LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Run single capture." << endl;
        if(m_Device->runSingleCapture(frame)) {
            LOG_INFO << "AcqThread::runImageCapture;\t\t" << "Single capture succeed !" << endl;
            LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "AcqThread::runImageCapture;" << "Single capture succeed !" << endl;
            saveImageCaptured(frame, i, imgOutput, imgPrefix);
        } else {
            LOG_ERROR << "AcqThread::runImageCapture;" << "Single capture failed !" << endl;
        }

    }

    //boost::this_thread::sleep(boost::posix_time::millisec(5));
 
    LOG_DEBUG << "AcqThread::runImageCapture;\t\t" << "Restarting camera in continuous mode..." << endl;

    if (!setCameraInContinousMode()) {
        throw runtime_error("Restart camera in continuous mode impossible");
    }

    LOG_DEBUG << "PERFORMANCE METRICS STOP runImageCapture";
}


/// ******************************************** FILE AND FOLDER MANAGEMENT

/// <summary>
/// Save image and set FITS data ASYNCHRONOUSLY
/// </summary>
/// <param name="frame"></param>
/// <param name="imgNum"></param>
/// <param name="outputType"></param>
/// <param name="imgPrefix"></param>
void AcqThread::saveImageCaptured(shared_ptr<Frame> source_frame, int imgNum, ImgFormat outputType, string imgPrefix) {
    LOG_DEBUG << "AcqThread::saveImageCaptured;\t\t"<< "AcqThread: SAVING IMAGE CAPTURED" << endl;
    //std::shared_ptr<cv::Mat> copiedImage = std::make_shared<cv::Mat>(source_frame->Image->clone());

    shared_ptr<Frame> frame = make_shared<Frame>(*source_frame);

    auto saveOperation = boost::async(boost::launch::async, [&]() -> bool {

        if (frame->Image->data) {

            string  YYYYMMDD = TimeDate::getYYYYMMDD(frame->mDate);

            if (buildAcquisitionDirectory(YYYYMMDD)) {

                string fileName = imgPrefix + "_" + TimeDate::getYYYYMMDDThhmmss(frame->mDate) + "_UT-" + Conversion::intToString(imgNum);

                switch (outputType) {

                case JPEG:
                {
                    switch (frame->mFormat) {

                    case CamPixFmt::MONO12:

                    {
                        cv::Mat temp;
                        frame->Image->copyTo(temp);
                        cv::Mat newMat = ImgProcessing::correctGammaOnMono12(temp, 2.2);
                        cv::Mat newMat2 = Conversion::convertTo8UC1(newMat);
                        SaveImg::saveJPEG(newMat2, mOutputDataPath + fileName);
                    }

                    break;

                    default:
                    {
                        cv::Mat temp;
                        frame->Image->copyTo(temp);
                        cv::Mat newMat = ImgProcessing::correctGammaOnMono8(temp, 2.2);
                        SaveImg::saveJPEG(newMat, mOutputDataPath + fileName);
                    }

                    }
                }

                break;

                case FITS:

                {

                    Fits2D newFits(mOutputDataPath);
                    newFits.loadKeys(mfkp, m_StationParam);
                    newFits.kSATURATE = frame->mSaturatedValue;
                    newFits.kCAMERA = m_Device->getDeviceName();
                    newFits.kGAINDB = frame->mGain;
                    newFits.kEXPOSURE = frame->mExposure / 1000000.0;
                    newFits.kONTIME = frame->mExposure / 1000000.0;
                    newFits.kELAPTIME = frame->mExposure / 1000000.0;
                    newFits.kDATEOBS = TimeDate::getIsoExtendedFormatDate(frame->mDate);

                    double  debObsInSeconds = frame->mDate.hours * 3600 + frame->mDate.minutes * 60 + frame->mDate.seconds;
                    double  julianDate = TimeDate::gregorianToJulian(frame->mDate);
                    double  julianCentury = TimeDate::julianCentury(julianDate);

                    newFits.kCRVAL1 = TimeDate::localSideralTime_2(julianCentury, frame->mDate.hours, frame->mDate.minutes, (int)frame->mDate.seconds, m_StationParam.SITELONG);
                    newFits.kCTYPE1 = "RA---ARC";
                    newFits.kCTYPE2 = "DEC--ARC";
                    newFits.kEQUINOX = 2000.0;

                    switch (frame->mFormat) {

                    case CamPixFmt::MONO12:

                    {

                        // Convert unsigned short type image in short type image.
                        shared_ptr<cv::Mat> newMat = make_shared<cv::Mat>(frame->Image->rows, frame->Image->cols, CV_16SC1, cv::Scalar(0));

                        // Set bzero and bscale for print unsigned short value in soft visualization.
                        newFits.kBZERO = 32768;
                        newFits.kBSCALE = 1;

                        unsigned short* ptr = NULL;
                        short* ptr2 = NULL;

                        for (int i = 0; i < frame->Image->rows; i++) {

                            ptr = frame->Image->ptr<unsigned short>(i);
                            ptr2 = newMat->ptr<short>(i);

                            for (int j = 0; j < frame->Image->cols; j++) {

                                if (ptr[j] - 32768 > 32767) {

                                    ptr2[j] = 32767;

                                }
                                else {

                                    ptr2[j] = ptr[j] - 32768;
                                }
                            }
                        }

                        // Create FITS image with BITPIX = SHORT_IMG (16-bits signed integers), pixel with TSHORT (signed short)
                        if (newFits.writeFits(newMat, S16, fileName))
                            LOG_DEBUG << "AcqThread::saveImageCaptured;\t\t" << "Fits saved in : " << mOutputDataPath << fileName << endl;

                    }

                    break;

                    default:

                    {

                        if (newFits.writeFits(frame->Image, UC8, fileName))
                            LOG_DEBUG << "AcqThread::saveImageCaptured;\t\t" << "Fits saved in : " << mOutputDataPath << fileName << endl;

                    }

                    }

                }

                break;

                }

            }
        }

        return true;
        });
}

/// <summary>
/// Build dataset folder structure
/// </summary>
/// <param name="YYYYMMDD"></param>
/// <returns></returns>
bool AcqThread::buildAcquisitionDirectory(string YYYYMMDD) {
    LOG_DEBUG << "AcqThread::buildAcquisitionDirectory" << endl;

    string root = m_DataParam.DATA_PATH + m_StationParam.STATION_NAME + "_" + YYYYMMDD + "/";

    string subDir = "captures/";
    string finalPath = root + subDir;

    mOutputDataPath = finalPath;
    LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "CompleteDataPath : " << mOutputDataPath << endl;

    path p(m_DataParam.DATA_PATH);
    path p1(root);
    path p2(root + subDir);

    // If DATA_PATH exists
    if (fs::exists(p)) {

        // If DATA_PATH/STATI ON_YYYYMMDD/ exists
        if (fs::exists(p1)) {

            // If DATA_PATH/STATION_YYYYMMDD/captures/ doesn't exists
            if (!fs::exists(p2)) {

                // If fail to create DATA_PATH/STATION_YYYYMMDD/captures/
                if (!fs::create_directory(p2)) {

                    LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create captures directory : " << p2.string() << endl;
                    return false;

                    // If success to create DATA_PATH/STATION_YYYYMMDD/captures/
                }
                else {

                    LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create captures directory : " << p2.string() << endl;
                    return true;

                }
            }

            // If DATA_PATH/STATION_YYYYMMDD/ doesn't exists
        }
        else {

            // If fail to create DATA_PATH/STATION_YYYYMMDD/
            if (!fs::create_directory(p1)) {

                LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create STATION_YYYYMMDD directory : " << p1.string() << endl;
                return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/
            }
            else {

                LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create STATION_YYYYMMDD directory : " << p1.string() << endl;

                // If fail to create DATA_PATH/STATION_YYYYMMDD/stack/
                if (!fs::create_directory(p2)) {

                    LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create captures directory : " << p2.string() << endl;
                    return false;

                    // If success to create DATA_PATH/STATION_YYYYMMDD/stack/
                }
                else {

                    LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create captures directory : " << p2.string() << endl;
                    return true;

                }
            }
        }

        // If DATA_PATH doesn't exists
    }
    else {

        // If fail to create DATA_PATH
        if (!fs::create_directory(p)) {

            LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create DATA_PATH directory : " << p.string() << endl;
            return false;

            // If success to create DATA_PATH
        }
        else {

            LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create DATA_PATH directory : " << p.string() << endl;

            // If fail to create DATA_PATH/STATION_YYYYMMDD/
            if (!fs::create_directory(p1)) {

                LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create STATION_YYYYMMDD directory : " << p1.string() << endl;
                return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/
            }
            else {

                LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create STATION_YYYYMMDD directory : " << p1.string() << endl;

                // If fail to create DATA_PATH/STATION_YYYYMMDD/captures/
                if (!fs::create_directory(p2)) {

                    LOG_ERROR << "AcqThread::buildAcquisitionDirectory;\t\t" << "Unable to create captures directory : " << p2.string() << endl;
                    return false;

                    // If success to create DATA_PATH/STATION_YYYYMMDD/captures/
                }
                else {

                    LOG_INFO << "AcqThread::buildAcquisitionDirectory;\t\t" << "Success to create captures directory : " << p2.string() << endl;
                    return true;

                }
            }
        }
    }

    return true;
}


// ********************************************* TIME MANAGEMENT

/// <summary>
/// Get how many seconds are passed counting from 0 (midnight) and now.
/// this function need to returns a unsigned long in order to avoid overflows. 
/// 
/// 24*60*60 = 86400
/// 
/// </summary>
/// <returns></returns>
unsigned long AcqThread::getNowInSeconds()
{
    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
    return getTimeInSeconds(time);
}

/// <summary>
/// Given a ptime returns how many seconds starting from midnight to the given timestamp
/// 
/// </summary>
/// <param name="time"></param>
/// <returns></returns>
unsigned long AcqThread::getTimeInSeconds(boost::posix_time::ptime time)
{
    string date = to_iso_extended_string(time);
    vector<int> intDate = TimeDate::getIntVectorFromDateString(date);
    return intDate.at(3) * 3600UL + intDate.at(4) * 60UL + intDate.at(5);
}

void AcqThread::setCurrentTimeDateAndSeconds(boost::posix_time::ptime time)
{
    m_CurrentTimeDate = TimeDate::Date(time);
    m_CurrentTimeInSeconds = m_CurrentTimeDate.hours * 3600 + m_CurrentTimeDate.minutes * 60 + (int)m_CurrentTimeDate.seconds;
}

void AcqThread::setCurrentTimeAndDate(boost::posix_time::ptime time) {
    string date = to_iso_extended_string(time);

    vector<int> intDate = TimeDate::getIntVectorFromDateString(date);
    string month = Conversion::intToString(intDate.at(1));
    
    if (month.size() == 1) 
        month = "0" + month;
    
    string day = Conversion::intToString(intDate.at(2));

    if (day.size() == 1) 
        day = "0" + day;

    mCurrentDate = Conversion::intToString(intDate.at(0)) + month + day;
    mCurrentTime = intDate.at(3) * 3600 + intDate.at(4) * 60 + intDate.at(5);
}

/// <summary>
/// Compute suntimes with ephemeris
/// </summary>
/// <returns></returns>
bool AcqThread::computeSunTimes() {
    LOG_DEBUG << "AcqThread::computeSunTimes" << endl;
    
    int sunriseStartH = 0, sunriseStartM = 0, sunriseStopH = 0, sunriseStopM = 0,
        sunsetStartH = 0, sunsetStartM = 0, sunsetStopH = 0, sunsetStopM = 0;

    LOG_DEBUG << "AcqThread::computeSunTimes;\t\t" << "LOCAL DATE      :  " << mCurrentDate << endl;

    if(m_CameraParam.ephem.EPHEMERIS_ENABLED) {
        LOG_INFO << "AcqThread::computeSunTimes;" << "COMPUTE SUN TIMES WITH EPHEMERIS" << endl;
        Ephemeris ephem1 = Ephemeris(mCurrentDate, m_CameraParam.ephem.SUN_HORIZON_1, m_StationParam.SITELONG, m_StationParam.SITELAT);

        if(!ephem1.computeEphemeris(sunriseStartH, sunriseStartM,sunsetStopH, sunsetStopM)) {

            return false;

        }

        Ephemeris ephem2 = Ephemeris(mCurrentDate, m_CameraParam.ephem.SUN_HORIZON_2, m_StationParam.SITELONG, m_StationParam.SITELAT );

        if(!ephem2.computeEphemeris(sunriseStopH, sunriseStopM,sunsetStartH, sunsetStartM)) {
            return false;
        }

    } else {
        LOG_INFO << "AcqThread::computeSunTimes;" << "COMPUTE DEFAULT SUN TIMES" << endl;
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

    LOG_DEBUG << "AcqThread::computeSunTimes;\t\t" << "SUNRISE         :  " << sunriseStartH << "H" << sunriseStartM << " - " << sunriseStopH << "H" << sunriseStopM << endl;
    LOG_DEBUG << "AcqThread::computeSunTimes;\t\t" << "SUNSET          :  " << sunsetStartH << "H" << sunsetStartM << " - " << sunsetStopH << "H" << sunsetStopM << endl;

    mStartSunriseTime = sunriseStartH * 3600UL + sunriseStartM * 60UL;
    mStopSunriseTime = sunriseStopH * 3600UL + sunriseStopM * 60UL;
    mStartSunsetTime = sunsetStartH * 3600UL + sunsetStartM * 60UL;
    mStopSunsetTime = sunsetStopH * 3600UL + sunsetStopM * 60UL;

    return true;
}


/// <summary>
/// time_in_seconds is in seconds and starts from 0 to 24*3600=86400
/// </summary>
/// <param name="time_in_seconds"></param>
/// <returns></returns>
bool AcqThread::isNight(unsigned long seconds)
{
    if ((seconds < mStartSunriseTime) || (seconds > mStopSunsetTime) )
        return true;

    return false;
}

bool AcqThread::isDay(unsigned long seconds)
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

bool AcqThread::isSunrise(unsigned long seconds)
{
    if ( seconds >= mStartSunriseTime && seconds <= mStopSunriseTime ) 
        return true;
    return false;             
}

bool AcqThread::isSunset(unsigned long seconds)
{
    if ( seconds >= mStartSunsetTime  && seconds <= mStopSunsetTime )
        return true;

     return false;
}

TimeMode AcqThread::getTimeMode(unsigned long seconds)
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




/// *********************************** METRICS
void AcqThread::metrics_ThreadLoop()
{
    double fps_limit = 0.0;
    double fps_floating_average = 0.0;
    double temperature = 0.0;

    m_LastMetricTimestamp = m_CurrentThreadLoopTime;
    

    //if runned a scheduled or regular acquisition, jump this fps value. is not valid.
    if (m_RunnedScheduledAcquisition || m_RunnedRegularAcquisition)
        return;
    
    //check fps value
    fps_limit = m_CameraParam.ACQ_FPS * FPS_MINIMUM_PERCENTAGE;

    if (m_CurrentFPS < fps_limit)
        LOG_WARNING << "AcqThread::metrics_ThreadLoop;" << "FPS is below " << fps_limit << "Hz " << m_CurrentFPS << endl;

    //calculate floating average for FPS circular buffer the same size as the frame buffer
    m_FPS_Sum += m_CurrentFPS;

    if (m_FPS_metrics.size() >= m_CircularBufferSize)
        m_FPS_Sum -= m_FPS_metrics.front();

    m_FPS_metrics.push_back(m_CurrentFPS);

    //UPDATE NODE EXPORTER METRICS
    boost::posix_time::time_duration metrics_time_elapsed = m_CurrentThreadLoopTime - m_LastMetricTimestamp;
    int64_t secTime = metrics_time_elapsed.total_seconds();

    if (secTime > METRICS_ACQUISITION_TIME)
    {

        fps_floating_average = m_FPS_Sum / (m_FPS_metrics.size());

        if (fps_floating_average < fps_limit)
            LOG_ERROR << "AcqThread::metrics_ThreadLoop;" << "FPS floating averageis below "<< fps_limit<<"Hz" << fps_floating_average << endl;

        temperature = m_Device->getTemperature();

        NodeExporterMetrics::GetInstance(m_StationParam.STATION_NAME).UpdateMetrics(fps_floating_average, tacq_metric, temperature, &nextSunrise, &nextSunset);
        NodeExporterMetrics::GetInstance(m_StationParam.STATION_NAME).WriteMetrics();
    }
}





unsigned long scheduleParamToSeconds(scheduleParam t)
{
    return t.hours * 3600 + t.minutes * 60 + t.seconds;
}


/// *********************************** SCHEDULED ACQUISITION

void AcqThread::selectFirstScheduledAcquisition()
{
    LOG_DEBUG << "AcqThread::selectFirstScheduledAcquisition" << endl;
    if (m_CameraParam.schcap.ACQ_SCHEDULE.size() != 0)
    {
        unsigned long now_seconds = TimeDate::getTimeDateToSeconds(m_CurrentThreadLoopTime);
        unsigned long min_seconds = 24UL * 3600UL + 60UL * 60UL + 60UL + 100UL; //get the greatest date ever in seconds... consider 100 Leap seconds... impossible

        bool found;

        // Search next acquisition
        size_t min_index = -1;

        //check hour
        for (size_t i = 0; i < m_CameraParam.schcap.ACQ_SCHEDULE.size(); i++)
        {
            unsigned long record_seconds = scheduleParamToSeconds(m_CameraParam.schcap.ACQ_SCHEDULE[i]);

            if (record_seconds < min_seconds)
            {
                found = true;
                min_seconds = record_seconds;
                min_index = i;
            }
        }

        //if not found, now is after the last scheduled capture, restart from 0
        if (!found)
            mNextAcqIndex = 0;

        mNextAcq = m_CameraParam.schcap.ACQ_SCHEDULE[mNextAcqIndex];

        LOG_DEBUG << "AcqThread::selectFirstScheduledAcquisition;" << "FIRST SCHCAP : " << mNextAcq.hours << "h" << mNextAcq.minutes << "m" << mNextAcq.seconds << "s" << endl;
    }
}

/// <summary>
/// Schedule must be ordered ascending from 00 to 23
/// </summary>
/// <param name="date"></param>
void AcqThread::selectNextAcquisitionSchedule() {
    LOG_DEBUG << "AcqThread::selectNextAcquisitionSchedule" << endl;

    if (m_CameraParam.schcap.ACQ_SCHEDULE.size() != 0)
    {
        unsigned long now_seconds = TimeDate::getTimeDateToSeconds(m_CurrentThreadLoopTime);
        unsigned long next_seconds = 24UL * 3600UL + 60UL * 60UL + 60UL + 100UL; //get the greatest date ever in seconds... consider 100 Leap seconds... impossible
        unsigned long min_seconds  = 24UL * 3600UL + 60UL * 60UL + 60UL + 100UL; //get the greatest date ever in seconds... consider 100 Leap seconds... impossible
        unsigned long record_seconds = 0UL;
        
        bool found;

        // Search next acquisition
        size_t min_index =-1;

        //check hour
        for (size_t i = 0; i < m_CameraParam.schcap.ACQ_SCHEDULE.size(); i++)
        {
            unsigned long record_seconds = scheduleParamToSeconds(m_CameraParam.schcap.ACQ_SCHEDULE[i]);

            if (record_seconds < min_seconds)
            {
                min_seconds = record_seconds;
                min_index = i;
            }

            //if the record is greater than now...
            if ( record_seconds >= now_seconds)
            {
                //if this record is lower then the last one found, we have found a new candidate
                if (record_seconds < next_seconds)
                {
                    next_seconds = record_seconds;
                    mNextAcqIndex = i;
                    found = true;
                }
            }
        }

        //if not found, now is after the last scheduled capture, restart from 0
        if (!found)
            mNextAcqIndex = min_index;

        mNextAcq = m_CameraParam.schcap.ACQ_SCHEDULE[mNextAcqIndex];

        if (LOG_SPAM_FRAME_STATUS)
        {
            LOG_DEBUG << "AcqThread::selectNextAcquisitionSchedule;" << "NEXT SCHCAP : " << mNextAcq.hours << "h" << mNextAcq.minutes << "m" << mNextAcq.seconds << "s" << endl;

            //log how long to the next one.
            unsigned long now_in_seconds = TimeDate::getTimeDateToSeconds(m_CurrentThreadLoopTime);
            unsigned long next_in_seconds = scheduleParamToSeconds(mNextAcq);

            unsigned long next = next_in_seconds - now_in_seconds;

            if (next > 0)
                LOG_DEBUG << "AcqThread::operator();" << "next : " << next << "[s]" << endl;
        }
    }
}

void AcqThread::resetContinuousMode()
{
    if (m_CurrentAcquisitionMode != EAcquisitionMode::CONTINUOUS)
    {
        LOG_INFO << "AcqThread::resetContinuousMode();" << "Set camera in continuous mode" << endl;
        setCameraInContinousMode();
    }
}

/// <summary>
/// Scheduled acquisition thread loop
///  - run next scheduled acquistion, if runned select the next one.
/// </summary>
void AcqThread::scheduledAcquisition_ThreadLoop()
{
    // ACQUISITION AT SCHEDULED TIME IS ENABLED.
    if (m_CameraParam.schcap.ACQ_SCHEDULE.size() != 0 && m_CameraParam.schcap.ACQ_SCHEDULE_ENABLED)
    {
        // It's time to run scheduled acquisition.
        if (mNextAcq.hours == m_CurrentTimeDate.hours &&
            mNextAcq.minutes == m_CurrentTimeDate.minutes &&
            (int)m_CurrentTimeDate.seconds == mNextAcq.seconds) {
            runScheduledAcquisition();

            //select next acquisition
            selectNextAcquisitionSchedule();
        }
    }
}

/// <summary>
/// Runs a scheduled acquisition
/// - reset frame buffer
/// - set single camera frame mode (if not already set)
/// - run image capture
/// </summary>
void  AcqThread::runScheduledAcquisition() {
    LOG_INFO << "AcqThread::runScheduledAcquisition;\t\t" << "Run scheduled acquisition." << endl;
    LOG_DEBUG << "PERFORMANCE METRICS START runScheduledAcquisition";
    //set mutex
    m_RunnedScheduledAcquisition = true;

    //reset framebuffer because time passed, free memory
    LOG_DEBUG << "AcqThread::runScheduledAcquisition;\t\t" << "Reset frame buffer" << endl;
    resetFrameBuffer();

    if (m_CurrentAcquisitionMode != EAcquisitionMode::SCHEDULED) 
    {
        LOG_INFO << "AcqThread::runScheduledAcquisition;\t\t" << "Setting single-frame mode" << endl;
        setCameraSingleFrameMode(EAcquisitionMode::SCHEDULED);
    }

    LOG_INFO << "AcqThread::runScheduledAcquisition;\t\t" << "Taking shot" << endl;
    runImageCapture(
        EAcquisitionMode::SCHEDULED,
        mNextAcq.rep,
        m_CameraParam.schcap.ACQ_SCHEDULE_OUTPUT,
        m_CameraParam.schcap.ACQ_SCHEDULE_PRFX
    );
    LOG_DEBUG << "PERFORMANCE METRICS END runScheduledAcquisition";
}




/// *********************************** REGULAR ACQUISITION

/// <summary>
/// Regular capture thread loop
/// </summary>
void AcqThread::regularAcquisition_ThreadLoop()
{
    // ACQUISITION AT REGULAR TIME INTERVAL IS ENABLED
    if (m_CameraParam.regcap.ACQ_REGULAR_ENABLED)
    {
        if (runNextRegularAcquisition())
        {
            LOG_INFO << "AcqThread::regularAcquisition_ThreadLoop;" << "============= Run regular acquisition because it is " << m_CurrentTimeMode << " and ACQ_REGULAR_MODE=" << m_CameraParam.regcap.ACQ_REGULAR_MODE << " =============" << endl;
            // Reset reference time BEFORE the exposure in case a long exposure has been done.
            m_LastRegularAcquisitionTimestamp = m_CurrentThreadLoopTime;

            runRegularAcquisition();
        }
    }
}

/// <summary>
/// Check if we need to run a regular capture
/// </summary>
/// <returns></returns>
bool AcqThread::runNextRegularAcquisition()
{
   
    boost::posix_time::time_duration regular_acq_time_elapsed = m_CurrentThreadLoopTime - m_LastRegularAcquisitionTimestamp;
    int64_t secTime = regular_acq_time_elapsed.total_seconds();

    if (LOG_SPAM_FRAME_STATUS)
        LOG_DEBUG << "AcqThread::runNextRegularAcquisition;" << "NEXT REGCAP : " << (int)(m_CameraParam.regcap.ACQ_REGULAR_CFG.interval - secTime) << "s" << endl;

    // Check it's time to run a regular capture.
    if (secTime >= m_CameraParam.regcap.ACQ_REGULAR_CFG.interval)
    {
        LOG_DEBUG << "AcqThread::runNextRegularAcquisition;" << "It's time to check if need to run a regular capture." << endl;
        if ((m_CurrentTimeMode == m_CameraParam.regcap.ACQ_REGULAR_MODE || m_CameraParam.regcap.ACQ_REGULAR_MODE == DAYNIGHT))
        {
            return true;
        }
        else {
            LOG_INFO << "AcqThread::runNextRegularAcquisition;" << "============= Jumped because it is " << m_CurrentTimeMode << " and ACQ_REGULAR_MODE=" << m_CameraParam.regcap.ACQ_REGULAR_MODE << " =============" << endl;
        }
    }

    return false;
}

/// <summary>
/// Runs a regular acquisition.
/// - reset frame buffer
/// - set camera in single frame mode (if not already set)
/// - run image capture
/// </summary>
void  AcqThread::runRegularAcquisition()
{
    LOG_DEBUG << "PERFORMANCE METRICS START runNextRegularAcquisition";

    LOG_INFO << "AcqThread::runRegularAcquisition;\t\t" << "Run regular acquisition." << endl;
    m_RunnedRegularAcquisition = true;
    //reset framebuffer because time passed, free memory
    LOG_DEBUG << "AcqThread::runRegularAcquisition;\t\t" << "Reset frame buffer" << endl;
    resetFrameBuffer();

    if (m_CurrentAcquisitionMode != EAcquisitionMode::REGULAR) {
        LOG_INFO << "AcqThread::runRegularAcquisition;\t\t" << "Setting single-frame mode" << endl;
        setCameraSingleFrameMode(EAcquisitionMode::REGULAR);
    }

    LOG_INFO << "AcqThread::runRegularAcquisition;\t\t" "Taking shot" << endl;
    runImageCapture(
        EAcquisitionMode::REGULAR,
        m_CameraParam.regcap.ACQ_REGULAR_CFG.rep,
        m_CameraParam.regcap.ACQ_REGULAR_OUTPUT,
        m_CameraParam.regcap.ACQ_REGULAR_PRFX
    );

    LOG_DEBUG << "PERFORMANCE METRICS END runNextRegularAcquisition";
}

