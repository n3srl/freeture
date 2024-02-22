#pragma once

/*
                                StackThread.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau -- GEOPS-UPSUD
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
* \file    StackThread.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    19/06/2014
* \brief   Stack frames.
*/
//header refactoring ok
#include "Commons.h"

#include <thread>
#include <memory>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>

#include "Fits.h"
#include "SParam.h"
#include "TimeDate.h"
#include "Frame.h"
#include "Logger.h"

// 
// #ifdef LINUX
//     #define BOOST_LOG_DYN_LINK 1
// #endif
// 
// #include <iostream>
// #include "EStackMeth.h"
// #include "Stack.h"
// #include "Fits.h"
// #include "Fits2D.h"
// #include "TimeDate.h"
// #include "EParser.h"
// #include <boost/filesystem.hpp>
// #include <boost/circular_buffer.hpp>
// #include <assert.h>
// #include "SParam.h"

namespace freeture
{
    class CfgParam;

    class StackThread {

    private:
        std::thread::id  m_ThreadID;

        boost::thread* mThread;
        bool            mustStop;
        boost::mutex    mustStopMutex;
        bool            isRunning;
        bool            interruptionStatus;
        boost::mutex    interruptionStatusMutex;

        boost::condition_variable* frameBuffer_condition;
        boost::mutex* frameBuffer_mutex;
        boost::circular_buffer<std::shared_ptr<Frame>>& frameBuffer;
        bool* stackSignal;
        boost::mutex* stackSignal_mutex;
        boost::condition_variable* stackSignal_condition;

        stationParam    mstp;
        fitskeysParam   mfkp;
        dataParam       mdp;
        stackParam      msp;
        CamPixFmt       mPixfmt;

        std::string completeDataPath;

    public:

        StackThread(bool*,
            boost::mutex*,
            boost::condition_variable*,
            boost::circular_buffer<std::shared_ptr<Frame>>&,
            boost::mutex*,
            boost::condition_variable*,
            std::shared_ptr<CfgParam>
        );

        ~StackThread(void);

        bool startThread();

        void stopThread();

        void operator()();

        bool getRunStatus();

        bool interruptThread();

    private:

        bool buildStackDataDirectory(TimeDate::Date date);

    };
}
