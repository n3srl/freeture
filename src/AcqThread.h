#pragma once
/*
                                AcqThread.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2016 Yoan Audureau, Chiara Marmo -- GEOPS-UPSUD
*                   (C) 2022-2024 Andrea Novati - N3 S.r.l.
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

/**
* \file    AcqThread.h
* \author  Yoan Audureau, Chiara Marmo -- GEOPS-UPSUD
* \version 1.0
* \date    03/10/2016
* \brief   Acquisition thread.
*/

//headers refactoring ok
#include "Commons.h"

#include <memory>
#include <thread>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/future.hpp>

#include <boost/circular_buffer.hpp>

#include "EParser.h"
#include "SParam.h"
#include "CameraSettings.h"
#include "TimeDate.h"
#include "Frame.h"
#include "EAcquisitionMode.h"
#include "Logger.h"

namespace freeture
{
    class DetThread;
    class StackThread;
    class Device;
    class ExposureControl;
    class CfgParam;


    class AcqThread
    {

    private:
        std::thread::id m_ThreadID;

        //COMMON THREADS ATTRIBUTES
 
        boost::mutex        mMustStopMutex;
        boost::thread*      mThread;                     // Acquisition thread.
        bool                mMustStop;              // Signal to stop thread.
        bool                mThreadTerminated;      // Terminated status of the thread.

        // Communication with the shared framebuffer.
        boost::condition_variable*      frameBuffer_condition;
        boost::mutex*                   frameBuffer_mutex;
        boost::circular_buffer<std::shared_ptr<Frame>>&  frameBuffer;

        // Communication with StackThread.
        bool m_StackThread_IsRunning = false;
        bool* stackSignal;
        boost::mutex* stackSignal_mutex;
        boost::condition_variable* stackSignal_condition;

        // Communication with DetThread.
        bool m_DetThread_IsRunning=false;
        bool* detSignal;
        boost::mutex* detSignal_mutex;
        boost::condition_variable* detSignal_condition;

        // ACQ THREAD ATTRIBUTES
        Device*                         m_Device;               // Device used for acquisition.
        bool                            m_ConnectionLost;       // Device connection is lost.
        ExposureControl*                pExpCtrl;               // Pointer on exposure time control object while sunrise and sunset.
        std::shared_ptr<DetThread>      pDetection;             // Pointer on detection thread in order to stop it or reset it when a regular capture occurs.
        std::shared_ptr<StackThread>    pStack;                 // Pointer on stack thread in order to save and reset a stack when a regular capture occurs.
        std::string                     mOutputDataPath;        // Dynamic location where to save data (regular captures etc...).
        std::string                     mCurrentDate;
        TimeDate::Date                  m_CurrentTimeDate;
        boost::posix_time::ptime        m_LastRegularAcquisitionTimestamp;
        boost::posix_time::ptime        m_LastMetricTimestamp;
        int                             mDeviceID;              // Index of the device to use.
        size_t                          mNextAcqIndex;
        long                            mStartSunriseTime;      // In seconds. CAN BE NEGATIVE
        long                            mStopSunriseTime;       // In seconds. CAN BE NEGATIVE
        long                            mStartSunsetTime;       // In seconds. CAN BE NEGATIVE
        long                            mStopSunsetTime;        // In seconds. CAN BE NEGATIVE
        
        unsigned long long              mFrameNumber;           // frame #
        bool                            cleanStatus = false;


        boost::posix_time::ptime m_CurrentThreadLoopTime;
        boost::posix_time::ptime m_PreviousThreadLoopTime;

        boost::posix_time::ptime m_RegularAcquisitionTime;   //used to store the current value used to trigger the capture. 
        boost::posix_time::ptime m_ScheduledAcquisitionTime; //used to store the current value used to trigger the capture. 

        unsigned long            mCurrentTime;           // m_CurrentThreadLoopTime but in seconds.
      


        // Parameters from configuration file.
        scheduleParam       mNextAcq;               // Next scheduled acquisition.
        stackParam          m_StackParam;
        stationParam        m_StationParam;
        detectionParam      m_DetectionParam;
        dataParam           m_DataParam;
        fitskeysParam       mfkp;

        cameraParam         m_CameraParam;
        framesParam         m_FramesParam;
        videoParam          m_VideoParam;

        CameraSettings m_CurrentCameraSettings;
        EAcquisitionMode m_CurrentAcquisitionMode;


        boost::circular_buffer<double>  m_FPS_metrics;


        //metrics management
        double m_CurrentFPS = 0.0;
        double t_FPS_metric = 0.0;
        double m_FPS_Sum = 0.0;

        size_t m_CircularBufferSize;
        std::shared_ptr<Frame> m_CurrentFrame;                  // current captured frame pointer

        TimeMode m_CurrentTimeMode;                             // Used to start/stop stack/detection thread on time mode changes
        TimeMode m_PreviousTimeMode;                            // Used to start/stop stack/detection thread on time mode changes

        EParser<TimeMode> time_mode_parser;
        bool m_RunnedRegularAcquisition = false;
        bool m_RunnedScheduledAcquisition = false;

        //EXPOSURE CONTROL

        // Exposure adjustment variables. used only if ACQ_AUTOEXPOSURE_ENABLED = true
        bool exposureControlStatus = false;
        bool exposureControlActive = false;

        boost::future<bool> m_SaveOperationFuture;
    public:

        AcqThread(boost::circular_buffer<std::shared_ptr<Frame>>&,
            boost::mutex*,
            boost::condition_variable*,
            bool*,
            boost::mutex*,
            boost::condition_variable*,
            bool*,
            boost::mutex*,
            boost::condition_variable*,
            std::shared_ptr<DetThread>,
            std::shared_ptr<StackThread>,
            std::shared_ptr<CfgParam>
        );

        ~AcqThread(void);


        //AUTOEXPOSURE
        void initAutoexposure();

        void resetTimeMode();

        //COMMON THREAD METHODS
        void operator()();

        bool startThread();

        void stopThread();

        // Return activity status.
        bool getThreadStatus();

        bool isNight(unsigned long);

        bool isDay(unsigned long);

        bool isSunset(unsigned long);

        bool isSunrise(unsigned long);

        bool isNight();

        bool isDay();

        bool isSunset();

        bool isSunrise();

        TimeMode getCurrentTimeMode();

        void updateTimeMode();
        TimeMode getTimeMode(unsigned long);

        bool connectionCheck();
        unsigned long getNowInSeconds();

        unsigned long getTimeInSeconds(boost::posix_time::ptime);

        void updateCurrentTimeAndDate();
        void setCurrentTimeAndDate(boost::posix_time::ptime);

        void updateTimeReferences();
        void updateCurrentTimeDateAndSeconds();
        void setCurrentTimeDateAndSeconds(boost::posix_time::ptime);

        bool selectNextDataSet();

        void resetTimeReferences();
        /// <summary>
        /// unlock condition variable for detection thread and notify_one
        /// </summary>
        /// <param name="YYYYMMDD"></param>
        /// <returns></returns>
        void startDetectionThread();

        /// <summary>
        /// lock condition variable for detection thread and interrupt thread
        /// </summary>
        void stopDetectionThread();

        /// <summary>
        ///  lock condition variable for stack thread and interrupt thread
        /// </summary>
        void stopStackThread();

        /// <summary>
        /// unlock condition variable for stack thread and notify_one
        /// </summary>
        /// <param name="img"></param>
        /// <param name="imgNum"></param>
        /// <param name="outputType"></param>
        /// <param name="imgPrefix"></param>
        void startStackThread();

        void resetFrameBuffer();

    private:
        void regularAcquisition_ThreadLoop();
        void scheduledAcquisition_ThreadLoop();
        void continuousAcquisition_ThreadLoop();
        void metrics_ThreadLoop();
        void metrics_reset();

        void continuousAcquisition_VIDEO();
        void continuousAcquisition_CAMERA();

        //internal device management

        bool selectNextRegularAcquisition();

        void resetContinuousMode();
        void resetSingleMode();
        void initializeLowFrequencyCameraSettings();
        bool setCameraInContinousMode();

        bool setCameraSingleFrameMode(EAcquisitionMode mode);

        void runScheduledAcquisition();
      
        void runRegularAcquisition();

        // Compute in seconds the sunrise start/stop times and the sunset start/stop times.
        bool computeSunTimes();

        // Build the directory where the data will be saved.
        bool buildAcquisitionDirectory(std::string);

        // Analyse the scheduled acquisition list to find the next one according to the current time.
        void selectNextAcquisitionSchedule();

        void selectFirstScheduledAcquisition();

        // Save a capture on disk.
        void saveImageCaptured(std::shared_ptr<Frame>, int, ImgFormat, std::string);

        // Run a regular or scheduled acquisition.
        void runImageCapture(EAcquisitionMode, int, ImgFormat, std::string);

        /// <summary>
        /// Prepare the device for a continuous or single-frame acquisition.
        /// 
        /// Compute sun times.
        /// 
        /// set exposure time (according to sun time)
        /// set gain (according to sun time)
        /// set fps (according to sun time AND acquisition mode)
        /// 
        /// initialize camera
        ///  - grabInitialization
        ///  - grabCleanse
        /// 
        /// start camera
        /// 
        /// </summary>
        /// <param name="mode"></param>
        /// <returns></returns>
        bool prepareAcquisitionOnDevice(EAcquisitionMode mode);

        void ApplyCameraSettingsToFrame(std::shared_ptr<Frame>);
    };
}