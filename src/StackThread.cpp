/*
                            StackThread.cpp

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
*   Last modified:      20/07/2015
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    StackThread.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    19/06/2014
* \brief   Stack frames.
*/
//header refactoring ok
#include "StackThread.h"

#include "CfgParam.h"

#include "NodeExporterMetrics.h"
#include "Stack.h"

#include <boost/date_time.hpp>


using namespace freeture;
using namespace std;

StackThread::StackThread(
    bool* sS,
    boost::mutex* sS_m,
    boost::condition_variable* sS_c,
    boost::circular_buffer<std::shared_ptr<Frame>>& fb,
    boost::mutex* fb_m,
    boost::condition_variable* fb_c,
    std::shared_ptr<CfgParam> cfg
) : 
    m_ThreadID(), mThread(nullptr), mustStop(false), mustStopMutex(), isRunning(false),interruptionStatus(false),
interruptionStatusMutex(), frameBuffer_condition(fb_c), frameBuffer_mutex(fb_m), frameBuffer(fb), stackSignal(sS), 
stackSignal_mutex(sS_m), stackSignal_condition(sS_c), mstp(cfg->getStationParam()), mfkp(cfg->getFitskeysParam()),
mdp(cfg->getDataParam()), msp(cfg->getStackParam()), mPixfmt(cfg->getCamParam().ACQ_FORMAT), completeDataPath("")
{
}

StackThread::~StackThread(void){

    LOG_INFO << "Cleaning ressources and deleting StackThread...";

    if(mThread!=NULL)
        delete mThread;

}

bool StackThread::startThread(){

    LOG_INFO << "Creating StackThread...";
    mThread = new boost::thread(boost::ref(*this));
    return true;
}

void StackThread::stopThread(){

    // Signal the thread to stop (thread-safe)
    mustStopMutex.lock();
    mustStop = true;
    mustStopMutex.unlock();

    if(mThread != NULL)
        while(mThread->timed_join(boost::posix_time::seconds(1)) == false)
            mThread->interrupt();

    interruptThread();


}

bool StackThread::interruptThread(){

    interruptionStatusMutex.lock();
    interruptionStatus = true;
    LOG_INFO << "StackThread interruption.";
    interruptionStatusMutex.unlock();
    return true;

}

bool StackThread::buildStackDataDirectory(TimeDate::Date date){

    namespace fs = boost::filesystem;
    string YYYYMMDD = TimeDate::getYYYYMMDD(date);
    string root = mdp.DATA_PATH + mstp.STATION_NAME + "_" + YYYYMMDD +"/";
    string subDir = "stacks/";
    string finalPath = root + subDir;
    completeDataPath = finalPath;

    if(YYYYMMDD == "00000000")
        return false;


    LOG_INFO << "Stacks data path : " << completeDataPath;

    path p(mdp.DATA_PATH);
    path p1(root);
    path p2(root + subDir);

    // If DATA_PATH exists
    if(fs::exists(p)){

        // If DATA_PATH/STATION_YYYYMMDD/ exists
        if(fs::exists(p1)){

            // If DATA_PATH/STATION_YYYYMMDD/stacks/ doesn't exists
            if(!fs::exists(p2)){

                // If fail to create DATA_PATH/STATION_YYYYMMDD/stacks/
                if(!fs::create_directory(p2)){

                    LOG_ERROR << "Unable to create stacks directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/stacks/
                }else{

                   LOG_INFO << "Success to create stacks directory : " << p2.string();
                   return true;

                }
            }

        // If DATA_PATH/STATION_YYYYMMDD/ doesn't exists
        }else{

            // If fail to create DATA_PATH/STATION_YYYYMMDD/
            if(!fs::create_directory(p1)){

                LOG_ERROR << "Unable to create STATION_YYYYMMDD directory : " << p1.string();
                return false;

            // If success to create DATA_PATH/STATION_YYYYMMDD/
            }else{

                LOG_INFO << "Success to create STATION_YYYYMMDD directory : " << p1.string();

                // If fail to create DATA_PATH/STATION_YYYYMMDD/stacks/
                if(!fs::create_directory(p2)){

                    LOG_ERROR << "Unable to create stacks directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/stacks/
                }else{

                    LOG_INFO << "Success to create stacks directory : " << p2.string();
                    return true;

                }
            }
        }

    // If DATA_PATH doesn't exists
    }else{

        // If fail to create DATA_PATH
        if(!fs::create_directory(p)){

            LOG_ERROR << "Unable to create DATA_PATH directory : " << p.string();
            return false;

        // If success to create DATA_PATH
        }else{

            LOG_INFO << "Success to create DATA_PATH directory : " << p.string();

            // If fail to create DATA_PATH/STATION_YYYYMMDD/
            if(!fs::create_directory(p1)){

                LOG_ERROR << "Unable to create STATION_YYYYMMDD directory : " << p1.string();
                return false;

            // If success to create DATA_PATH/STATION_YYYYMMDD/
            }else{

                LOG_INFO << "Success to create STATION_YYYYMMDD directory : " << p1.string();

                // If fail to create DATA_PATH/STATION_YYYYMMDD/stacks/
                if(!fs::create_directory(p2)){

                    LOG_ERROR << "Unable to create stacks directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/stacks/
                }else{

                    LOG_INFO << "Success to create stacks directory : " << p2.string();
                    return true;

                }
            }
        }
    }

    return true;
}

bool StackThread::getRunStatus(){

    return isRunning;

}

/// <summary>
/// 
/// </summary>
void StackThread::operator()(){
    m_ThreadID = std::this_thread::get_id();

    Logger::GetLogger()->setLogThread(LogThread::STACK_THREAD, m_ThreadID);

    bool stop = false;
    isRunning = true;

    LOG_INFO << "\n";
    LOG_INFO << "==============================================";
    LOG_INFO << "============== Start stack thread ============";
    LOG_INFO << "==============================================";

    try {
        unsigned long interval = msp.STACK_INTERVAL * 1000;
        do {
            string cDate ="";
            try {



                // Thread is sleeping...
                if (interval) {
                    LOG_DEBUG << "operator();" << "Thread sleeping for " << interval << "[ms]";
                    boost::this_thread::sleep(boost::posix_time::millisec(interval));
                    LOG_DEBUG << "operator();" << "Create a stack to accumulate n frames.";
                }

                // Create a stack to accumulate n frames.
                Stack frameStack = Stack(mdp.FITS_COMPRESSION_METHOD, mfkp, mstp);

                // First reference date.
                boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
                cDate = to_simple_string(time);
                string dateDelimiter = ".";
                string refDate = cDate.substr(0, cDate.find(dateDelimiter));

                long secTime = 0;

                do {
                    /// Wait new frame from AcqThread.
                    if (LOG_SPAM_FRAME_STATUS)
                        LOG_DEBUG << "operator();" << "This is STACK THREAD waiting ACQUISTION THREAD for a new frame.";
                    // Communication with AcqThread. Wait for a new frame.
                    boost::mutex::scoped_lock lock(*stackSignal_mutex);
                    while(!(*stackSignal)) stackSignal_condition->wait(lock);
                    *stackSignal = false;
                    lock.unlock();
                    LOG_DEBUG << "operator();" << "Unlocked";


                    double t = (double)cv::getTickCount();

                    // Check interruption signal from AcqThread.
                    bool forceToSave = false;
                    interruptionStatusMutex.lock();
                    if(interruptionStatus) forceToSave = true;
                    interruptionStatusMutex.unlock();

                    if(!forceToSave){

                        // Fetch last frame grabbed.
                        boost::mutex::scoped_lock lock2(*frameBuffer_mutex);
                        if(frameBuffer.size() == 0) {
                            throw "SHARED CIRCULAR BUFFER SIZE = 0 -> STACK INTERRUPTION.";
                        }
                        shared_ptr<Frame> newFrame = frameBuffer.back();
                        lock2.unlock();

                        // Add the new frame to the stack.
                        frameStack.addFrame(newFrame);

                        t = (((double)cv::getTickCount() - t)/ cv::getTickFrequency())*1000;
                        if (LOG_SPAM_FRAME_STATUS){
                            LOG_INFO << "[ TIME STACK ] : " << setprecision(5) << fixed << t << " ms" ;
                        }

                    }else{

                        // Interruption is active.
                        LOG_INFO << "Interruption status : " << forceToSave;

                        // Interruption operations terminated. Rest interruption signal.
                        interruptionStatusMutex.lock();
                        interruptionStatus = false;
                        interruptionStatusMutex.unlock();

                        break;

                    }

                    time = boost::posix_time::microsec_clock::universal_time();
                    cDate = to_simple_string(time);
                    string nowDate = cDate.substr(0, cDate.find(dateDelimiter));
                    boost::posix_time::ptime t1(boost::posix_time::time_from_string(refDate));
                    boost::posix_time::ptime t2(boost::posix_time::time_from_string(nowDate));
                    boost::posix_time::time_duration td = t2 - t1;
                    secTime = td.total_seconds();
                    if (LOG_SPAM_FRAME_STATUS)
                        LOG_DEBUG << "operator();" << "NEXT STACK : " << (int)(msp.STACK_TIME - secTime) << "s";

                } while(secTime <= msp.STACK_TIME);

                // Stack finished. Save it.
                LOG_DEBUG << "operator();" << "Stack finished. Save it.";

                if(buildStackDataDirectory(frameStack.getDateFirstFrame())) {

                    if(!frameStack.saveStack(completeDataPath, msp.STACK_MTHD, msp.STACK_REDUCTION)){

                        LOG_ERROR << "operator();" << "Fail to save stack.";

                    }

                    LOG_INFO << "Stack saved : " << completeDataPath;

                }else{

                    LOG_ERROR << "operator();" << "Fail to build stack directory. " << completeDataPath;

                }

            } catch(const boost::thread_interrupted&) {

                LOG_INFO << "operator();" << "Stack thread INTERRUPTED";
            }

            // Get the "must stop" state (thread-safe)
            mustStopMutex.lock();
            stop = mustStop;
            mustStopMutex.unlock();

            NodeExporterMetrics::GetInstance(mstp.STATION_NAME).UpdateMetrics(completeDataPath,cDate);
            NodeExporterMetrics::GetInstance(mstp.STATION_NAME).WriteMetrics();

        } while(!stop);

    }catch(const char * msg){

        LOG_ERROR << "operator();" << msg;

    }catch(exception& e){

        LOG_ERROR << "operator();" << e.what();

    }

    isRunning = false;

    LOG_INFO << "StackThread ended.";

}

