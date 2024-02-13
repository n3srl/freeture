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

#include "TimeDate.h"
#include "Frame.h"


#include "SParam.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>


namespace freeture
{
    class DetThread;
    class StackThread;
    class Device;
    class ExposureControl;

    class AcqThread {

    private:

        bool                mMustStop;              // Signal to stop thread.
        boost::mutex        mMustStopMutex;
        boost::thread* mThread;               // Acquisition thread.
        bool                mThreadTerminated;      // Terminated status of the thread.
        Device* mDevice;               // Device used for acquisition.
        int                 mDeviceID;              // Index of the device to use.
        scheduleParam       mNextAcq;               // Next scheduled acquisition.
        int                 mNextAcqIndex;
        DetThread* pDetection;            // Pointer on detection thread in order to stop it or reset it when a regular capture occurs.
        StackThread* pStack;                // Pointer on stack thread in order to save and reset a stack when a regular capture occurs.
        ExposureControl* pExpCtrl;              // Pointer on exposure time control object while sunrise and sunset.
        std::string              mOutputDataPath;        // Dynamic location where to save data (regular captures etc...).
        std::string              mCurrentDate;
        int                 mStartSunriseTime;      // In seconds.
        int                 mStopSunriseTime;       // In seconds.
        int                 mStartSunsetTime;       // In seconds.
        int                 mStopSunsetTime;        // In seconds.
        int                 mCurrentTime;           // In seconds.

        // Parameters from configuration file.
        stackParam          msp;
        stationParam        mstp;
        detectionParam      mdtp;
        cameraParam         mcp;
        dataParam           mdp;
        fitskeysParam       mfkp;
        framesParam         mfp;
        videoParam          mvp;

        // Communication with the shared framebuffer.
        boost::condition_variable* frameBuffer_condition;
        boost::mutex* frameBuffer_mutex;
        boost::circular_buffer<Frame>* frameBuffer;

        // Communication with DetThread.
        bool* stackSignal;
        boost::mutex* stackSignal_mutex;
        boost::condition_variable* stackSignal_condition;

        // Communication with StackThread.
        bool* detSignal;
        boost::mutex* detSignal_mutex;
        boost::condition_variable* detSignal_condition;

    public:

        AcqThread(boost::circular_buffer<Frame>* fb,
            boost::mutex* fb_m,
            boost::condition_variable* fb_c,
            bool* sSignal,
            boost::mutex* sSignal_m,
            boost::condition_variable* sSignal_c,
            bool* dSignal,
            boost::mutex* dSignal_m,
            boost::condition_variable* dSignal_c,
            DetThread* detection,
            StackThread* stack,
            int                                 cid,
            dataParam                           dp,
            stackParam                          sp,
            stationParam                        stp,
            detectionParam                      dtp,
            cameraParam                         acq,
            framesParam                         fp,
            videoParam                          vp,
            fitskeysParam                       fkp,
            freeture::Device* device);

        ~AcqThread(void);

        void operator()();

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
        void stopThread();

        bool startThread();

        void stopDetectionThread();
        void stopStackThread();
        void resetFrameBuffer();

        void notifyDetectionThread();

        // Return activity status.
        bool getThreadStatus();


        // Added
        bool buildCameraInContinousMode(bool);

    private:
        bool cleanStatus = false;


        // Compute in seconds the sunrise start/stop times and the sunset start/stop times.
        bool computeSunTimes();

        // Build the directory where the data will be saved.
        bool buildAcquisitionDirectory(std::string YYYYMMDD);

        // Analyse the scheduled acquisition list to find the next one according to the current time.
        void selectNextAcquisitionSchedule(TimeDate::Date date);

        // Save a capture on disk.
        void saveImageCaptured(Frame& img, int imgNum, ImgFormat outputType, std::string imgPrefix);

        // Run a regular or scheduled acquisition.
        void runImageCapture(int imgNumber, int imgExposure, int imgGain, CamPixFmt imgFormat, ImgFormat imgOutput, std::string imgPrefix);

        // Prepare the device for a continuous acquisition.
        bool prepareAcquisitionOnDevice();
    };
}