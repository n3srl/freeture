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

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>

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
        boost::circular_buffer<std::shared_ptr<Frame>>  frameBuffer;

        // Communication with DetThread.
        bool* stackSignal;
        boost::mutex* stackSignal_mutex;
        boost::condition_variable* stackSignal_condition;

        // Communication with StackThread.
        bool* detSignal;
        boost::mutex* detSignal_mutex;
        boost::condition_variable* detSignal_condition;

        // ACQ THREAD ATTRIBUTES
        Device*                         m_Device;               // Device used for acquisition.
        ExposureControl*                pExpCtrl;               // Pointer on exposure time control object while sunrise and sunset.
        std::shared_ptr<DetThread>      pDetection;             // Pointer on detection thread in order to stop it or reset it when a regular capture occurs.
        std::shared_ptr<StackThread>    pStack;                 // Pointer on stack thread in order to save and reset a stack when a regular capture occurs.
        std::string                     mOutputDataPath;        // Dynamic location where to save data (regular captures etc...).
        std::string                     mCurrentDate;
        std::string                     cDate;
        std::string                     refDate;
        int                             mDeviceID;              // Index of the device to use.
        int                             mNextAcqIndex;
        int                             mStartSunriseTime;      // In seconds.
        int                             mStopSunriseTime;       // In seconds.
        int                             mStartSunsetTime;       // In seconds.
        int                             mStopSunsetTime;        // In seconds.
        int                             mCurrentTime;           // In seconds.
        unsigned long long              mFrameNumber;           // frame #
        bool                            cleanStatus = false;
      

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

    public:

        AcqThread(boost::circular_buffer<std::shared_ptr<Frame>>,
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

        //COMMON THREAD METHODS
        void operator()();

        bool startThread();

        void stopThread();

        // Return activity status.
        bool getThreadStatus();

        bool isNight(int);

        bool isDay(int);

        bool isSunset(int);

        bool isSunrise(int);

        bool isNight();

        bool isDay();

        bool isSunset();

        bool isSunrise();

        TimeMode getCurrentTimeMode();

        TimeMode getTimeMode(int);

        int getNowInSeconds();

        int getTimeInSeconds(boost::posix_time::ptime);

        void startDetectionThread();

        void stopDetectionThread();

        void stopStackThread();

        void startStackThread();

        void resetFrameBuffer();

        void notifyDetectionThread();

        //internal device management
        bool setCameraInContinousMode();

        bool setCameraSingleFrameMode(EAcquisitionMode mode);

    private:
        
        void runScheduledAcquisition(TimeDate::Date);
      
        void runRegularAcquisition();

        // Compute in seconds the sunrise start/stop times and the sunset start/stop times.
        bool computeSunTimes();

        // Build the directory where the data will be saved.
        bool buildAcquisitionDirectory(std::string YYYYMMDD);

        // Analyse the scheduled acquisition list to find the next one according to the current time.
        void selectNextAcquisitionSchedule(TimeDate::Date date);

        // Save a capture on disk.
        void saveImageCaptured(std::shared_ptr<Frame> img, int imgNum, ImgFormat outputType, std::string imgPrefix);

        // Run a regular or scheduled acquisition.
        void runImageCapture(EAcquisitionMode mode, int imgNumber, ImgFormat imgOutput, std::string imgPrefix);

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