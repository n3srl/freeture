#pragma once
/*
                                DetThread.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
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
*   Last modified:      20/10/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    DetThread.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
* \brief   Detection thread.
*/
//header refactoring ok
#include "Commons.h"

#include <memory>
#include <string>
#include <thread>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>

#include "Fits.h"
#include "SParam.h"
#include "TimeDate.h"
#include "Frame.h"

// #include "SMTPClient.h"
// #include <iterator>
// #include "Fits.h"
// #include "Fits2D.h"
// #include "TimeDate.h"
// #include "Fits3D.h"
// #include "Stack.h"
// #include "Detection.h"
// #include "DetectionTemporal.h"
// #include "DetectionTemplate.h"
// #include "EDetMeth.h"
// #include <boost/circular_buffer.hpp>
// #include <boost/filesystem.hpp>
// #include "ESmtpSecurity.h"
// #include "SParam.h"
// 
// using namespace boost::filesystem;
// using namespace boost::posix_time;

namespace freeture
{
    class Detection;
    class CfgParam;

    class DetThread
    {

    private:
        std::thread::id m_ThreadID;

        boost::thread* pThread;                   // Pointer on detection thread.
        Detection* pDetMthd;                  // Pointer on detection method.
        bool                            mMustStop;
        boost::mutex                    mMustStopMutex;
        std::string                     mStationName;               // Name of the station              (parameter from configuration file).
        CamPixFmt                       mFormat;                    // Acquisition bit depth            (parameter from configuration file).
        Fits                            mFitsHeader;
        bool                            mIsRunning;                 // Detection thread running status.
        bool                            mWaitFramesToCompleteEvent;
        int                             mNbWaitFrames;
        std::string                     mCfgPath;
        std::string                     mEventPath;                 // Path of the last detected event.
        TimeDate::Date                  mEventDate;                 // Date of the last detected event.
        int                             mNbDetection;               // Number of detection.
        bool                            mInterruptionStatus;
        boost::mutex                    mInterruptionStatusMutex;
        boost::circular_buffer<std::shared_ptr<Frame>>& frameBuffer;
        boost::mutex* frameBuffer_mutex;
        boost::condition_variable* frameBuffer_condition;
        bool* detSignal;
        boost::mutex* detSignal_mutex;
        boost::condition_variable* detSignal_condition;
        std::string                          mCurrentDataSetLocation;
        std::vector<std::pair<std::string, int>>        mDetectionResults;
        bool                            mForceToReset;

        detectionParam                  mdtp;
        dataParam                       mdp;
        mailParam                       mmp;
        fitskeysParam                   mfkp;
        stationParam                    mstp;
        int                             mNbFramesAround; // Number of frames to keep around an event.


    public:

        DetThread(boost::circular_buffer<std::shared_ptr<Frame>>&,
            boost::mutex*,
            boost::condition_variable*,
            bool*,
            boost::mutex*,
            boost::condition_variable*,
            std::shared_ptr<CfgParam>
           );

        ~DetThread();

        void operator()();

        bool startThread();

        void stopThread();

        bool buildEventDataDirectory();

        /**
        * Save an event in the directory "events".
        *
        * @param firstEvPosInFB First frame's number of the event.
        * @param lastEvPosInFB Last frame's number of the event.
        * @return Success to save an event.
        */
        bool saveEventData(int firstEvPosInFB, int lastEvPosInFB);

        /**
        * Run status of detection thread.
        *
        * @return Is running or not.
        */
        bool getRunStatus();

        /**
        * Get detection method used by detection thread.
        *
        * @return Detection method.
        */
        Detection* getDetMethod();

        /**
        * Interrupt detection thread.
        *
        */
        void interruptThread();

        void updateDetectionReport() {

            if (mCurrentDataSetLocation != "") {

                mDetectionResults.push_back(std::pair<std::string, int>(mCurrentDataSetLocation, mNbDetection));
                mNbDetection = 0;

            }
        };

        void setCurrentDataSet(std::string location) {

            mCurrentDataSetLocation = location;

        };

    };
}
