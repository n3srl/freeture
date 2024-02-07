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

/**
* \file    AcqThread.cpp
* \author  Yoan Audureau, Chiara Marmo -- GEOPS-UPSUD
* \version 1.0
* \date    21/01/2015
* \brief   Acquisition thread.
*/

#include "Constants.h"

#include "AcqThread.h"

#include "NodeExporterMetrics.h"

using namespace cv;
using namespace freeture;

boost::log::sources::severity_logger< LogSeverityLevel >  AcqThread::logger;

AcqThread::Init AcqThread::initializer;

AcqThread::AcqThread(   boost::circular_buffer<Frame>       *fb,
                        boost::mutex                        *fb_m,
                        boost::condition_variable           *fb_c,
                        bool                                *sSignal,
                        boost::mutex                        *sSignal_m,
                        boost::condition_variable           *sSignal_c,
                        bool                                *dSignal,
                        boost::mutex                        *dSignal_m,
                        boost::condition_variable           *dSignal_c,
                        DetThread                           *detection,
                        StackThread                         *stack,
                        int                                 cid,
                        dataParam                           dp,
                        stackParam                          sp,
                        stationParam                        stp,
                        detectionParam                      dtp,
                        cameraParam                         acq,
                        framesParam                         fp,
                        videoParam                          vp,
                        fitskeysParam                       fkp,
                        Device*                             device )
{
    std::cout << "AcqThread::AcqThread"<<std::endl;
    CameraDeviceManager& manager = CameraDeviceManager::Get();
    frameBuffer             = fb;
    frameBuffer_mutex       = fb_m;
    frameBuffer_condition   = fb_c;
    stackSignal             = sSignal;
    stackSignal_mutex       = sSignal_m;
    stackSignal_condition   = sSignal_c;
    detSignal               = dSignal;
    detSignal_mutex         = dSignal_m;
    detSignal_condition     = dSignal_c;
    pDetection              = detection;
    pStack                  = stack;
    mThread                 = NULL;
    mMustStop               = false;
    mDevice                 = manager.getDevice();
    mThreadTerminated       = false;
    mNextAcqIndex           = 0;
    pExpCtrl                = NULL;
    mDeviceID               = cid;
    mdp                     = dp;
    msp                     = sp;
    mstp                    = stp;
    mdtp                    = dtp;
    mcp                     = acq;
    mvp                     = vp;
    mfp                     = fp;

}

AcqThread::~AcqThread(void){
    std::cout << "AcqThread::~AcqThread"<<std::endl;
    if(mDevice != NULL)
        delete mDevice;

    if(mThread != NULL)
        delete mThread;

    if(pExpCtrl != NULL)
        delete pExpCtrl;

}

void AcqThread::stopThread(){
    std::cout << "AcqThread::stopThread"<<std::endl;
    mMustStopMutex.lock();
    mMustStop = true;
    mMustStopMutex.unlock();

    if(mThread != NULL)
        while(mThread->timed_join(boost::posix_time::seconds(2)) == false)
            mThread->interrupt();

}

bool AcqThread::buildCameraInContinousMode(bool rebuild = false) {
    // CREATE CAMERA
    
    mDevice->Setup(mcp, mfp, mvp, mDeviceID);
    if (mDevice->mCam == nullptr) {
            
        if(!mDevice->createCamera()) {
            std::cout << "CREATE CAMERA ERROR" << std::endl;
            return false;
        }
    } 

    
    // Prepare continuous acquisition.
    if(!prepareAcquisitionOnDevice()) {
        std::cout << "FAIL ON PREPARE AQ DEVICE" << std::endl;
        return false;
    }

    return true;
}

bool AcqThread::startThread() {
    std::cout << "AcqThread::startThread(" << std::endl;
    
    
    if (!buildCameraInContinousMode()) {
        return false;
    }
        

    // Create acquisition thread.
    mThread = new boost::thread(boost::ref(*this));
    
    return true;

}

bool AcqThread::getThreadStatus(){
    //std::cout << "AcqThread::getThreadStatus"<<std::endl;
    return mThreadTerminated;
}

void AcqThread::stopDetectionThread()
{
    if ( pDetection != nullptr ) {
        boost::mutex::scoped_lock lock(*detSignal_mutex);
        *detSignal = false;
        lock.unlock();
        
        // Force interruption.
        std::cout << "Sending interruption signal to detection process... " << std::endl;
        pDetection->interruptThread();
    }
}

void AcqThread::stopStackThread()
{
    if ( pStack != nullptr ) {
        boost::mutex::scoped_lock lock(*stackSignal_mutex);
        *stackSignal = false;
        lock.unlock();

        // Force interruption.
        std::cout << "Send interruption signal to stack " << std::endl;
        pStack->interruptThread();
    }
}

void AcqThread::resetFrameBuffer()
{
    std::cout << "Cleaning frameBuffer..." << std::endl;
    boost::mutex::scoped_lock lock(*frameBuffer_mutex);
    frameBuffer->clear();
    lock.unlock();
    cleanStatus = true;
}

void AcqThread::notifyDetectionThread() {
    if(pDetection != NULL)
    {
        boost::mutex::scoped_lock lock2(*detSignal_mutex);
        *detSignal = true;
        detSignal_condition->notify_one();
        lock2.unlock();
    }
}

void AcqThread::operator()()
{

    bool stop = false;
    std::cout << "========== START ACQUISITION THREAD ==========" << std::endl;
    BOOST_LOG_SCOPED_THREAD_TAG("LogName", "ACQ_THREAD");
    BOOST_LOG_SEV(logger,notification) << "\n";
    BOOST_LOG_SEV(logger,notification) << "==============================================";
    BOOST_LOG_SEV(logger,notification) << "========== START ACQUISITION THREAD ==========";
    BOOST_LOG_SEV(logger,notification) << "==============================================";

    try {

        // Search next acquisition according to the current time.
        selectNextAcquisitionSchedule(TimeDate::splitIsoExtendedDate(to_iso_extended_string(boost::posix_time::microsec_clock::universal_time())));

        // Exposure adjustment variables.
        bool exposureControlStatus = false;
        bool exposureControlActive = false;
        
        // If exposure can be set on the input device.
        if(mDevice->getExposureStatus())
        {
            if (pExpCtrl!=nullptr)
                delete pExpCtrl;
            else pExpCtrl = new ExposureControl( mcp.EXPOSURE_CONTROL_FREQUENCY,
                                            mcp.EXPOSURE_CONTROL_SAVE_IMAGE,
                                            mcp.EXPOSURE_CONTROL_SAVE_INFOS,
                                            mdp.DATA_PATH,
                                            mstp.STATION_NAME);    
        }

        TimeMode previousTimeMode =  getTimeMode(getNowInSeconds());

        /// Acquisition process.
        do {

            // Location of a video or frames if input type is FRAMES or VIDEO.
            std::string location = "";

            // Load videos file or frames directory if input type is FRAMES or VIDEO
            if(!mDevice->loadNextCameraDataSet(location))
                break;
            
            if(pDetection != NULL)
                pDetection->setCurrentDataSet(location);

            // Reference time to compute interval between regular captures.
            std::string cDate = to_simple_string(boost::posix_time::microsec_clock::universal_time());
            std::string refDate = cDate.substr(0, cDate.find("."));

            do {

                // Container for the grabbed image.
                Frame newFrame;

                // Time counter of grabbing a frame.
                double tacq = (double)getTickCount();
                double fps;
                std::vector<int> nextSunset;
                std::vector<int> nextSunrise;

                // Grab a frame.
                if(mDevice->runContinuousCapture(newFrame)) {

                    if (LOG_FRAME_STATUS)
                    {
                        BOOST_LOG_SEV(logger, normal)   << "============= FRAME " << newFrame.mFrameNumber << " ============= ";
                        std::cout                            << "============= FRAME " << newFrame.mFrameNumber << " ============= " << std::endl;
                    }

                    // If camera type in input is FRAMES or VIDEO.
                    if(mDevice->mVideoFramesInput) {
                        //std::cout << "mDevice->mVideoFramesInput" << std::endl;
                        // Push the new frame in the framebuffer.
                        boost::mutex::scoped_lock lock(*frameBuffer_mutex);
                        frameBuffer->push_back(newFrame);
                        lock.unlock();

                        // Notify detection thread.
                        notifyDetectionThread();

                        // Slow down the time in order to give more time to the detection process.
                        int twait = 100;
                        if(mvp.INPUT_TIME_INTERVAL == 0 && mfp.INPUT_TIME_INTERVAL > 0)
                            twait = mfp.INPUT_TIME_INTERVAL;
                        else if(mvp.INPUT_TIME_INTERVAL > 0 && mfp.INPUT_TIME_INTERVAL == 0)
                            twait = mvp.INPUT_TIME_INTERVAL;
                        #ifdef WINDOWS
                            Sleep(twait);
                        #else
                            #ifdef LINUX
                                usleep(twait * 1000);
                            #endif
                        #endif

                    }else {
                        // Get current time in seconds.
                        int currentTimeInSec = newFrame.mDate.hours * 3600 + newFrame.mDate.minutes * 60 + (int)newFrame.mDate.seconds;

                        // Detect day or night.
                        TimeMode currentTimeMode = getTimeMode(currentTimeInSec);

                        

                        // If exposure control is not active, the new frame can be shared with others threads.
                        if(!exposureControlStatus) {

                            // Push the new frame in the framebuffer.
                            boost::mutex::scoped_lock lock(*frameBuffer_mutex);
                            frameBuffer->push_back(newFrame);
                            lock.unlock();

                            // Notify detection thread.
                            if(pDetection != NULL) {
                                if(previousTimeMode != currentTimeMode && mdtp.DET_MODE != DAYNIGHT) {
                                    std::cout << "ERROR HERE Notify detection thread." << std::endl;
                                    BOOST_LOG_SEV(logger, notification) << "TimeMode has changed ! ";
                                    boost::mutex::scoped_lock lock(*detSignal_mutex);
                                    *detSignal = false;
                                    lock.unlock();
                                    std::cout << "Send interruption signal to detection process " << std::endl;
                                    pDetection->interruptThread();

                                }else if(mdtp.DET_MODE == currentTimeMode || mdtp.DET_MODE == DAYNIGHT) {

                                    boost::mutex::scoped_lock lock2(*detSignal_mutex);
                                    *detSignal = true;
                                    detSignal_condition->notify_one();
                                    lock2.unlock();

                                }
                            }

                            // Notify stack thread.
                            if(pStack != NULL) {

                                // TimeMode has changed.
                                if(previousTimeMode != currentTimeMode && msp.STACK_MODE != DAYNIGHT) {
                                    //std::cout << "ERROR HERE Notify stack thread " << std::endl;
                                    BOOST_LOG_SEV(logger, notification) << "TimeMode has changed ! ";
                                    boost::mutex::scoped_lock lock(*stackSignal_mutex);
                                    *stackSignal = false;
                                    lock.unlock();

                                    // Force interruption.
                                    std::cout << "Send interruption signal to stack " << std::endl;
                                    pStack->interruptThread();

                                }else if(msp.STACK_MODE == currentTimeMode || msp.STACK_MODE == DAYNIGHT) {

                                    boost::mutex::scoped_lock lock3(*stackSignal_mutex);
                                    *stackSignal = true;
                                    stackSignal_condition->notify_one();
                                    lock3.unlock();

                                }
                            }

                            cleanStatus = false;
                        }else {

                            // Exposure control is active, the new frame can't be shared with others threads.
                            if(!cleanStatus) {

                                // If stack process exists.
                                stopStackThread();

                                // If detection process exists
                                stopDetectionThread();
                                
                                // Reset framebuffer.
                                resetFrameBuffer();
                                
                            }

                        }

                        previousTimeMode = currentTimeMode;

                        // Adjust exposure time.
                        if(pExpCtrl != NULL && exposureControlActive)
                            exposureControlStatus = pExpCtrl->controlExposureTime(mDevice, newFrame.mImg, newFrame.mDate, mdtp.MASK, mDevice->mMinExposureTime, mcp.ACQ_FPS);

                        // Get current date YYYYMMDD.
                        std::string currentFrameDate =   TimeDate::getYYYYMMDD(newFrame.mDate);

                        // If the date has changed, sun ephemeris must be updated.
                        if(currentFrameDate != mCurrentDate)
                        {
                            BOOST_LOG_SEV(logger, notification) << "Date has changed. Former Date is " << mCurrentDate << ". New Date is " << currentFrameDate << "." ;
                            computeSunTimes();
                        }
                        // Acquisition at regular time interval is enabled.
                        if(mcp.regcap.ACQ_REGULAR_ENABLED && !mDevice->mVideoFramesInput) {
                            //std::cout << "TRY REGULAR ACQ " << std::endl;
                            cDate = to_simple_string(boost::posix_time::microsec_clock::universal_time());
                            std::string nowDate = cDate.substr(0, cDate.find("."));

                            boost::posix_time::ptime t1(boost::posix_time::time_from_string(refDate));
                            boost::posix_time::ptime t2(boost::posix_time::time_from_string(nowDate));

                            boost::posix_time::time_duration td = t2 - t1;
                            long secTime = td.total_seconds();

                            if (LOG_FRAME_STATUS)
                                std::cout << "NEXT REGCAP : " << (int)(mcp.regcap.ACQ_REGULAR_CFG.interval - secTime) << "s" <<  std::endl;

                            // Check it's time to run a regular capture.
                            if(secTime >= mcp.regcap.ACQ_REGULAR_CFG.interval) {

                                // Current time is after the sunset stop and before the sunrise start = NIGHT
                                if((currentTimeMode == NIGHT) && (mcp.regcap.ACQ_REGULAR_MODE == NIGHT || mcp.regcap.ACQ_REGULAR_MODE == DAYNIGHT)) {

                                        BOOST_LOG_SEV(logger, notification) << "Run regular acquisition.";
                                        std::cout << "Run regular acquisition." << std::endl;
                                        runImageCapture(    mcp.regcap.ACQ_REGULAR_CFG.rep,
                                                            mcp.regcap.ACQ_REGULAR_CFG.exp,
                                                            mcp.regcap.ACQ_REGULAR_CFG.gain,
                                                            mcp.regcap.ACQ_REGULAR_CFG.fmt,
                                                            mcp.regcap.ACQ_REGULAR_OUTPUT,
                                                            mcp.regcap.ACQ_REGULAR_PRFX);

                                // Current time is between sunrise start and sunset stop = DAY
                                }else if(currentTimeMode == DAY && (mcp.regcap.ACQ_REGULAR_MODE == DAY || mcp.regcap.ACQ_REGULAR_MODE == DAYNIGHT)) {

                                    BOOST_LOG_SEV(logger, notification) << "Run regular acquisition.";
                                    saveImageCaptured(newFrame, 0, mcp.regcap.ACQ_REGULAR_OUTPUT, mcp.regcap.ACQ_REGULAR_PRFX);

                                }

                                // Reset reference time in case a long exposure has been done.
                                cDate = to_simple_string(boost::posix_time::microsec_clock::universal_time());
                                refDate = cDate.substr(0, cDate.find("."));

                            }

                        }
                        // Acquisiton at scheduled time is enabled.
                        if(mcp.schcap.ACQ_SCHEDULE.size() != 0 && mcp.schcap.ACQ_SCHEDULE_ENABLED && !mDevice->mVideoFramesInput) {

                            int next = (mNextAcq.hours * 3600 + mNextAcq.min * 60 + mNextAcq.sec) - (newFrame.mDate.hours * 3600 + newFrame.mDate.minutes * 60 + newFrame.mDate.seconds);

                            if(next < 0) {
                                next = (24 * 3600) - (newFrame.mDate.hours * 3600 + newFrame.mDate.minutes * 60 + newFrame.mDate.seconds) + (mNextAcq.hours * 3600 + mNextAcq.min * 60 + mNextAcq.sec);
                                if (LOG_FRAME_STATUS)
                                    std::cout << "next : " << next << std::endl;
                            }

                            std::vector<int>tsch = TimeDate::HdecimalToHMS(next/3600.0);
                            if (LOG_FRAME_STATUS)
                                std::cout << "NEXT SCHCAP : " << tsch.at(0) << "h" << tsch.at(1) << "m" << tsch.at(2) << "s" <<  std::endl;

                            // It's time to run scheduled acquisition.
                            if( mNextAcq.hours == newFrame.mDate.hours &&
                                mNextAcq.min == newFrame.mDate.minutes &&
                                (int)newFrame.mDate.seconds == mNextAcq.sec) {

                                CamPixFmt format;
                                format = mNextAcq.fmt;

                                runImageCapture(    mNextAcq.rep,
                                                    mNextAcq.exp,
                                                    mNextAcq.gain,
                                                    format,
                                                    mcp.schcap.ACQ_SCHEDULE_OUTPUT,
                                                    "");

                                // Update mNextAcq
                                selectNextAcquisitionSchedule(newFrame.mDate);

                            }else {

                                // The current time has elapsed.
                                if(newFrame.mDate.hours > mNextAcq.hours) {

                                   selectNextAcquisitionSchedule(newFrame.mDate);

                                }else if(newFrame.mDate.hours == mNextAcq.hours) {

                                    if(newFrame.mDate.minutes > mNextAcq.min) {

                                        selectNextAcquisitionSchedule(newFrame.mDate);

                                    }else if(newFrame.mDate.minutes == mNextAcq.min) {

                                        if(newFrame.mDate.seconds > mNextAcq.sec) {

                                            selectNextAcquisitionSchedule(newFrame.mDate);

                                        }

                                    }

                                }

                            }

                        }

                        // Check sunrise and sunset time.
                        if( (isSunrise(currentTimeInSec) || isSunset(currentTimeInSec)) 
                                && !mDevice->mVideoFramesInput) {
                            exposureControlActive = true;

                        } else {

                            // Print time before sunrise.
                            if(!isSunrise(currentTimeInSec)) {
                                nextSunrise.clear();
                                if(currentTimeInSec < mStartSunriseTime)
                                    nextSunrise = TimeDate::HdecimalToHMS((mStartSunriseTime - currentTimeInSec) / 3600.0);
                                if(currentTimeInSec > mStopSunsetTime)
                                    nextSunrise = TimeDate::HdecimalToHMS(((24*3600 - currentTimeInSec) + mStartSunriseTime ) / 3600.0);
                                if (LOG_FRAME_STATUS)
                                    std::cout << "NEXT SUNRISE : " << nextSunrise.at(0) << "h" << nextSunrise.at(1) << "m" << nextSunrise.at(2) << "s" << std::endl;
                            }

                            // Print time before sunset.
                            if(!isSunset())
                            {
                                nextSunset.clear();
                                nextSunset = TimeDate::HdecimalToHMS((mStartSunsetTime - currentTimeInSec) / 3600.0);
                                if (LOG_FRAME_STATUS)
                                    std::cout << "NEXT SUNSET : " << nextSunset.at(0) << "h" << nextSunset.at(1) << "m" << nextSunset.at(2) << "s" << std::endl;

                            }

                            // Reset exposure time when sunrise or sunset is finished.
                            if(exposureControlActive) {

                                // In DAYTIME : Apply minimum available exposure time.
                                if(isDay(currentTimeInSec)) 
                                {

                                    BOOST_LOG_SEV(logger, notification) << "Apply day exposure time : " << mDevice->getDayExposureTime();
                                    mDevice->setCameraDayExposureTime();
                                    BOOST_LOG_SEV(logger, notification) << "Apply day exposure time : " << mDevice->getDayGain();
                                    mDevice->setCameraDayGain();

                                // In NIGHTTIME : Apply maximum available exposure time.
                                } 
                                else if(isNight(currentTimeInSec)) 
                                {

                                    BOOST_LOG_SEV(logger, notification) << "Apply night exposure time." << mDevice->getNightExposureTime();
                                    mDevice->setCameraNightExposureTime();
                                    BOOST_LOG_SEV(logger, notification) << "Apply night exposure time." << mDevice->getNightGain();
                                    mDevice->setCameraNightGain();
                                }
                            }

                            exposureControlActive = false;
                            exposureControlStatus = false;
                        }

                    }

                }

                tacq = (((double)getTickCount() - tacq)/getTickFrequency())*1000;
                fps = (1.0/(tacq/1000.0)) ;
                if (LOG_FRAME_STATUS)
                    std::cout << " [ TIME ACQ ] : " << tacq << " ms   ~cFPS("  << fps << ")" <<  std::endl;
                BOOST_LOG_SEV(logger, normal) << " [ TIME ACQ ] : " << tacq << " ms";

                mMustStopMutex.lock();
                stop = mMustStop;
                mMustStopMutex.unlock();

                NodeExporterMetrics::GetInstance().UpdateMetrics( fps, tacq,&nextSunrise, &nextSunset );
                NodeExporterMetrics::GetInstance().WriteMetrics();

            } while(stop == false && !mDevice->getCameraStatus());

            // Reset detection process to prepare the analyse of a new data set.
            if(pDetection != NULL) 
            {
                pDetection->getDetMethod()->resetDetection(true);
                pDetection->getDetMethod()->resetMask();
                pDetection->updateDetectionReport();
                if(!pDetection->getRunStatus())
                    break;

            }

            // Clear framebuffer.
            resetFrameBuffer();
        } while(mDevice->getCameraDataSetStatus() && stop == false);

    }catch(const boost::thread_interrupted&){

        BOOST_LOG_SEV(logger,notification) << "AcqThread ended.";
        std::cout << "AcqThread ended." <<std::endl;

    }catch(std::exception& e){

        std::cout << "An exception occured : " << e.what() << std::endl;
        BOOST_LOG_SEV(logger, critical) << "An exception occured : " << e.what();

    }catch(const char * msg) {

        std::cout << std::endl << msg << std::endl;

    }

    mDevice->stopCamera();

    mThreadTerminated = true;

    std::cout << "Acquisition Thread TERMINATED." << std::endl;
    BOOST_LOG_SEV(logger,notification) << "Acquisition Thread TERMINATED";

}

void AcqThread::selectNextAcquisitionSchedule(TimeDate::Date date) {
    std::cout << "AcqThread::selectNextAcquisitionSchedule"<<std::endl;

    if(mcp.schcap.ACQ_SCHEDULE.size() != 0){

        // Search next acquisition
        for(int i = 0; i < mcp.schcap.ACQ_SCHEDULE.size(); i++){

            if(date.hours < mcp.schcap.ACQ_SCHEDULE.at(i).hours){

               mNextAcqIndex = i;
               break;

            }else if(date.hours == mcp.schcap.ACQ_SCHEDULE.at(i).hours){

                if(date.minutes < mcp.schcap.ACQ_SCHEDULE.at(i).min){

                    mNextAcqIndex = i;
                    break;

                }else if(date.minutes == mcp.schcap.ACQ_SCHEDULE.at(i).min){

                    if(date.seconds < mcp.schcap.ACQ_SCHEDULE.at(i).sec){

                        mNextAcqIndex = i;
                        break;

                    }
                }
            }
        }

        mNextAcq = mcp.schcap.ACQ_SCHEDULE.at(mNextAcqIndex);

    }

}

bool AcqThread::buildAcquisitionDirectory(std::string YYYYMMDD){
    std::cout << "AcqThread::buildAcquisitionDirectory"<<std::endl;

    namespace fs = boost::filesystem;
    std::string root = mdp.DATA_PATH + mstp.STATION_NAME + "_" + YYYYMMDD +"/";

    std::string subDir = "captures/";
    std::string finalPath = root + subDir;

    mOutputDataPath = finalPath;
    BOOST_LOG_SEV(logger,notification) << "CompleteDataPath : " << mOutputDataPath;

    path p(mdp.DATA_PATH);
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

                    BOOST_LOG_SEV(logger,critical) << "Unable to create captures directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/captures/
                }else{

                   BOOST_LOG_SEV(logger,notification) << "Success to create captures directory : " << p2.string();
                   return true;

                }
            }

        // If DATA_PATH/STATION_YYYYMMDD/ doesn't exists
        }else{

            // If fail to create DATA_PATH/STATION_YYYYMMDD/
            if(!fs::create_directory(p1)){

                BOOST_LOG_SEV(logger,fail) << "Unable to create STATION_YYYYMMDD directory : " << p1.string();
                return false;

            // If success to create DATA_PATH/STATION_YYYYMMDD/
            }else{

                BOOST_LOG_SEV(logger,notification) << "Success to create STATION_YYYYMMDD directory : " << p1.string();

                // If fail to create DATA_PATH/STATION_YYYYMMDD/stack/
                if(!fs::create_directory(p2)){

                    BOOST_LOG_SEV(logger,critical) << "Unable to create captures directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/stack/
                }else{

                    BOOST_LOG_SEV(logger,notification) << "Success to create captures directory : " << p2.string();
                    return true;

                }
            }
        }

    // If DATA_PATH doesn't exists
    }else{

        // If fail to create DATA_PATH
        if(!fs::create_directory(p)){

            BOOST_LOG_SEV(logger,fail) << "Unable to create DATA_PATH directory : " << p.string();
            return false;

        // If success to create DATA_PATH
        }else{

            BOOST_LOG_SEV(logger,notification) << "Success to create DATA_PATH directory : " << p.string();

            // If fail to create DATA_PATH/STATION_YYYYMMDD/
            if(!fs::create_directory(p1)){

                BOOST_LOG_SEV(logger,fail) << "Unable to create STATION_YYYYMMDD directory : " << p1.string();
                return false;

            // If success to create DATA_PATH/STATION_YYYYMMDD/
            }else{

                BOOST_LOG_SEV(logger,notification) << "Success to create STATION_YYYYMMDD directory : " << p1.string();

                // If fail to create DATA_PATH/STATION_YYYYMMDD/captures/
                if(!fs::create_directory(p2)){

                    BOOST_LOG_SEV(logger,critical) << "Unable to create captures directory : " << p2.string();
                    return false;

                // If success to create DATA_PATH/STATION_YYYYMMDD/captures/
                }else{

                    BOOST_LOG_SEV(logger,notification) << "Success to create captures directory : " << p2.string();
                    return true;

                }
            }
        }
    }

    return true;
}

void AcqThread::runImageCapture(int imgNumber, int imgExposure, int imgGain, CamPixFmt imgFormat, ImgFormat imgOutput, std::string imgPrefix) {
    std::cout << "AcqThread::runImageCapture"<<std::endl;
    // Stop camera
    mDevice->stopCamera();

    // Stop stack process.
    if(pStack != NULL){

        boost::mutex::scoped_lock lock(*stackSignal_mutex);
        *stackSignal = false;
        lock.unlock();

        // Force interruption.
        BOOST_LOG_SEV(logger, notification) << "Send reset signal to stack. ";
        pStack->interruptThread();

    }

    // Stop detection process.
    if(pDetection != NULL){

        boost::mutex::scoped_lock lock(*detSignal_mutex);
        *detSignal = false;
        lock.unlock();
        BOOST_LOG_SEV(logger, notification) << "Send reset signal to detection process. ";
        pDetection->interruptThread();

    }

    // Reset framebuffer.
    BOOST_LOG_SEV(logger, notification) << "Cleaning frameBuffer...";
    boost::mutex::scoped_lock lock(*frameBuffer_mutex);
    frameBuffer->clear();
    lock.unlock();

    for(int i = 0; i < imgNumber; i++) {

        BOOST_LOG_SEV(logger, notification) << "Prepare capture n° " << i;

        // Configuration for single capture.
        Frame frame;
        BOOST_LOG_SEV(logger, notification) << "Exposure time : " << imgExposure;
        frame.mExposure  = imgExposure;
        BOOST_LOG_SEV(logger, notification) << "Gain : " << imgGain;
        frame.mGain = imgGain;
        EParser<CamPixFmt> format;
        BOOST_LOG_SEV(logger, notification) << "Format : " << format.getStringEnum(imgFormat);
        frame.mFormat = imgFormat;

        frame.mFps = 30;

        if(mcp.ACQ_RES_CUSTOM_SIZE) {
            frame.mHeight = mcp.ACQ_HEIGHT;
            frame.mWidth = mcp.ACQ_WIDTH;

            frame.mStartX = mcp.ACQ_STARTX;
            frame.mStartY = mcp.ACQ_STARTY;
        }

        // Run single capture.
        BOOST_LOG_SEV(logger, notification) << "Run single capture.";
        if(mDevice->runSingleCapture(frame)) {

            BOOST_LOG_SEV(logger, notification) << "Single capture succeed !";
            std::cout << "Single capture succeed !" << std::endl;
            saveImageCaptured(frame, i, imgOutput, imgPrefix);

        }else{

            BOOST_LOG_SEV(logger, fail) << "Single capture failed !";

        }

    }

    #ifdef WINDOWS
        Sleep(1000);
    #else
        #ifdef LINUX
            sleep(5);
        #endif
    #endif

    std::cout << "Restarting camera in continuous mode..." << std::endl;
    BOOST_LOG_SEV(logger, notification) << "Restarting camera in continuous mode...";

    if (!buildCameraInContinousMode(true)) {
        throw "Restart camera in continuos mode impossible";
    }

}

void AcqThread::saveImageCaptured(Frame &img, int imgNum, ImgFormat outputType, std::string imgPrefix) {
    std::cout << "AcqThread: SAVING IMAGE CAPTURED" << std::endl;
    if(img.mImg.data) {

        std::string  YYYYMMDD = TimeDate::getYYYYMMDD(img.mDate);

        if(buildAcquisitionDirectory(YYYYMMDD)) {

            std::string fileName = imgPrefix + "_" + TimeDate::getYYYYMMDDThhmmss(img.mDate) + "_UT-" + Conversion::intToString(imgNum);

            switch(outputType) {

                case JPEG :

                    {

                        switch(img.mFormat) {

                            case MONO12 :

                                {

                                    Mat temp;
                                    img.mImg.copyTo(temp);
                                    Mat newMat = ImgProcessing::correctGammaOnMono12(temp, 2.2);
                                    Mat newMat2 = Conversion::convertTo8UC1(newMat);
                                    SaveImg::saveJPEG(newMat2, mOutputDataPath + fileName);

                                }

                                break;

                            default :

                                {

                                    Mat temp;
                                    img.mImg.copyTo(temp);
                                    Mat newMat = ImgProcessing::correctGammaOnMono8(temp, 2.2);
                                    SaveImg::saveJPEG(newMat, mOutputDataPath + fileName);

                                }

                        }
                    }

                    break;

                case FITS :

                    {

                        Fits2D newFits(mOutputDataPath);
                        newFits.loadKeys(mfkp, mstp);
                        newFits.kGAINDB = img.mGain;
                        newFits.kEXPOSURE = img.mExposure/1000000.0;
                        newFits.kONTIME = img.mExposure/1000000.0;
                        newFits.kELAPTIME = img.mExposure/1000000.0;
                        newFits.kDATEOBS = TimeDate::getIsoExtendedFormatDate(img.mDate);

                        double  debObsInSeconds = img.mDate.hours*3600 + img.mDate.minutes*60 + img.mDate.seconds;
                        double  julianDate      = TimeDate::gregorianToJulian(img.mDate);
                        double  julianCentury   = TimeDate::julianCentury(julianDate);

                        newFits.kCRVAL1 = TimeDate::localSideralTime_2(julianCentury, img.mDate.hours, img.mDate.minutes, (int)img.mDate.seconds, mstp.SITELONG);
                        newFits.kCTYPE1 = "RA---ARC";
                        newFits.kCTYPE2 = "DEC--ARC";
                        newFits.kEQUINOX = 2000.0;

                        switch(img.mFormat) {

                            case MONO12 :

                                {

                                    // Convert unsigned short type image in short type image.
                                    Mat newMat = Mat(img.mImg.rows, img.mImg.cols, CV_16SC1, Scalar(0));

                                    // Set bzero and bscale for print unsigned short value in soft visualization.
                                    newFits.kBZERO = 32768;
                                    newFits.kBSCALE = 1;

                                    unsigned short *ptr = NULL;
                                    short *ptr2 = NULL;

                                    for(int i = 0; i < img.mImg.rows; i++){

                                        ptr = img.mImg.ptr<unsigned short>(i);
                                        ptr2 = newMat.ptr<short>(i);

                                        for(int j = 0; j < img.mImg.cols; j++){

                                            if(ptr[j] - 32768 > 32767){

                                                ptr2[j] = 32767;

                                            }else{

                                                ptr2[j] = ptr[j] - 32768;
                                            }
                                        }
                                    }

                                    // Create FITS image with BITPIX = SHORT_IMG (16-bits signed integers), pixel with TSHORT (signed short)
                                    if(newFits.writeFits(newMat, S16, fileName))
                                        std::cout << ">> Fits saved in : " << mOutputDataPath << fileName << std::endl;

                                }

                                break;

                            default :

                                {

                                   if(newFits.writeFits(img.mImg, UC8, fileName))
                                        std::cout << ">> Fits saved in : " << mOutputDataPath << fileName << std::endl;

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
    std::string date = to_iso_extended_string(time);
    std::vector<int> intDate = TimeDate::getIntVectorFromDateString(date);
    return intDate.at(3) * 3600 + intDate.at(4) * 60 + intDate.at(5);
}

bool AcqThread::computeSunTimes() {
    std::cout << "AcqThread::computeSunTimes"<<std::endl;

    int sunriseStartH = 0, sunriseStartM = 0, sunriseStopH = 0, sunriseStopM = 0,
        sunsetStartH = 0, sunsetStartM = 0, sunsetStopH = 0, sunsetStopM = 0;

    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
    std::string date = to_iso_extended_string(time);
    std::vector<int> intDate = TimeDate::getIntVectorFromDateString(date);

    std::string month = Conversion::intToString(intDate.at(1));
    if(month.size() == 1) month = "0" + month;
    std::string day = Conversion::intToString(intDate.at(2));
    if(day.size() == 1) day = "0" + day;
    mCurrentDate = Conversion::intToString(intDate.at(0)) + month + day;
    mCurrentTime = intDate.at(3) * 3600 + intDate.at(4) * 60 + intDate.at(5);

    std::cout << "LOCAL DATE      :  " << mCurrentDate << " " << time << std::endl;

    if(mcp.ephem.EPHEMERIS_ENABLED) {

        Ephemeris ephem1 = Ephemeris(mCurrentDate, mcp.ephem.SUN_HORIZON_1, mstp.SITELONG, mstp.SITELAT);

        if(!ephem1.computeEphemeris(sunriseStartH, sunriseStartM,sunsetStopH, sunsetStopM)) {

            return false;

        }

        Ephemeris ephem2 = Ephemeris(mCurrentDate, mcp.ephem.SUN_HORIZON_2, mstp.SITELONG, mstp.SITELAT );

        if(!ephem2.computeEphemeris(sunriseStopH, sunriseStopM,sunsetStartH, sunsetStartM)) {

            return false;

        }

    }else {

        sunriseStartH = mcp.ephem.SUNRISE_TIME.at(0);
        sunriseStartM = mcp.ephem.SUNRISE_TIME.at(1);

        double intpart1 = 0;
        double fractpart1 = modf((double)mcp.ephem.SUNRISE_DURATION/3600.0 , &intpart1);

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

        sunsetStartH = mcp.ephem.SUNSET_TIME.at(0);
        sunsetStartM = mcp.ephem.SUNSET_TIME.at(1);

        double intpart3 = 0;
        double fractpart3 = modf((double)mcp.ephem.SUNSET_DURATION/3600.0 , &intpart3);

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

    mStartSunriseTime = (sunriseStartH * 3600 + sunriseStartM * 60);
    mStopSunriseTime = (sunriseStopH * 3600 + sunriseStopM * 60);
    mStartSunsetTime = (sunsetStartH * 3600 + sunsetStartM * 60);
    mStopSunsetTime = (sunsetStopH * 3600 + sunsetStopM * 60);

    std::cout << "SUNRISE         :  " << sunriseStartH << "H" << sunriseStartM << " - " << sunriseStopH << "H" << sunriseStopM << " ("<< mStartSunriseTime <<","<<mStopSunriseTime <<")" << std::endl;
    std::cout << "SUNSET          :  " << sunsetStartH << "H" << sunsetStartM << " - " << sunsetStopH << "H" << sunsetStopM << " ("<< mStartSunsetTime <<","<<mStopSunsetTime <<")" << std::endl;
    std::cout << "NOW             :  " << getCurrentTimeModeString() << " ("<<mCurrentTime<<")"<< std::endl;
    

    return true;

}



// mCurrentTime is in seconds and starts from 0 to 24*3600=86400
// int is a wrong container even if on x64 4 bytes are used to make a int.
// better to use a specific datetime container eg. boost
// using minors because if is exactly the second of sunrise/sunset start it is not considered night
bool AcqThread::isNight(int currentTimeInSec)
{
    if ((currentTimeInSec < mStartSunriseTime) || (currentTimeInSec > mStopSunsetTime) 
            && currentTimeInSec > 0 
            && mStartSunriseTime > 0
            && mStopSunsetTime > 0 ) 
        return true;

    return false;
}

bool AcqThread::isDay(int seconds)
{
    if ( seconds > mStopSunriseTime && seconds < mStartSunsetTime
            && seconds > 0 
            && mStartSunriseTime > 0
            && mStopSunsetTime > 0 )
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
    if (   seconds >= mStartSunriseTime 
        && seconds <= mStopSunriseTime
        && seconds > 0 
        && mStartSunriseTime > 0
        && mStopSunriseTime > 0 ) 
        return true;
    return false;             
}

bool AcqThread::isSunset(int seconds){
    if(    seconds >= mStartSunsetTime 
        && seconds <= mStopSunsetTime
        && seconds > 0 
        && mStartSunsetTime > 0
        && mStopSunsetTime > 0 
        )
        return true;

     return false;
}

TimeMode AcqThread::getTimeMode(int seconds)
{
    if (isNight(seconds)){
        //std::cout << "AcqThread::getTimeMode(" << seconds << ") => " << "NIGHT" << std::endl;
        return TimeMode::NIGHT;
    }

    if (isSunrise(seconds)){
        //std::cout << "AcqThread::getTimeMode(" << seconds << ") => " << "SUNRISE" << std::endl;
        return TimeMode::SUNRISE;
    }
    if (isSunset(seconds)){
        //std::cout << "AcqThread::getTimeMode(" << seconds << ") => " << "SUNSET" << std::endl;
        return TimeMode::SUNSET;
    }

    if (isDay(seconds)){
        //std::cout << "AcqThread::getTimeMode(" << seconds << ") => " << "DAY" << std::endl;
        return TimeMode::DAY;
    }
    //std::cout << "AcqThread::getTimeMode(" << seconds << ") => " << "NONE" << std::endl;
    return TimeMode::NONE;
}

std::string AcqThread::getCurrentTimeModeString()
{
    TimeMode mode = getCurrentTimeMode();
    switch (mode)
    {
    case TimeMode::DAY:
        return "DAY";
    case TimeMode::NIGHT:
        return "NIGHT";
    case TimeMode::SUNRISE:
        return "SUNRISE";
    case TimeMode::SUNSET:
        return "SUNSET";
    case TimeMode::NONE:
        return "NONE";
        break;
    
    default:
    return "";
        break;
    }
} 

TimeMode AcqThread::getCurrentTimeMode()
{
    if (isNight()){
        //std::cout << "AcqThread::getCurrentTimeMode() => " << "NIGHT" << std::endl;
        return TimeMode::NIGHT;
    }

    if (isSunrise()){
        //std::cout << "AcqThread::getCurrentTimeMode() => " << "SUNRISE" << std::endl;
        return TimeMode::SUNRISE;
    }
    if (isSunset()){
        //std::cout << "AcqThread::getCurrentTimeMode() => " << "SUNSET" << std::endl;
        return TimeMode::SUNSET;
    }

    if (isDay()){
        //std::cout << "AcqThread::getCurrentTimeMode() => " << "DAY" << std::endl;
        return TimeMode::DAY;
    }
    //std::cout << "AcqThread::getCurrentTimeMode() => " << "NONE" << std::endl;
    return TimeMode::NONE;
}

bool AcqThread::prepareAcquisitionOnDevice() 
{
    
    std::cout << "AcqThread::prepareAcquisitionOnDevice" << std::endl;
    
    // SET SIZE
    if(!mDevice->setCameraSize())
        return false;
    // SET FORMAT
    if(!mDevice->setCameraPixelFormat())
        return false;

    // LOAD GET BOUNDS
    mDevice->getCameraFPSBounds();
    mDevice->getCameraExposureBounds();
    mDevice->getCameraGainBounds();

    // Get Sunrise start/stop, Sunset start/stop. ---
    computeSunTimes();

    // CHECK SUNRISE AND SUNSET TIMES.
    if(isNight()) 
    {
        std::cout << "*************************** SET NIGHT AUTO NO" << std::endl;
        BOOST_LOG_SEV(logger, notification) << "DAYTIME         :  NO";
        BOOST_LOG_SEV(logger, notification) << "AUTO EXPOSURE   :  NO";
        BOOST_LOG_SEV(logger, notification) << "EXPOSURE TIME   :  " << mDevice->getNightExposureTime();
        BOOST_LOG_SEV(logger, notification) << "GAIN            :  " << mDevice->getNightGain();

        if(!mDevice->setCameraNightExposureTime())
            return false;

        if(!mDevice->setCameraNightGain())
           return false;

    }
    else 
        if(isDay()) {
            std::cout << "*************************** SET DAY AUTO NO" << std::endl;
            BOOST_LOG_SEV(logger, notification) << "DAYTIME         :  YES";
            BOOST_LOG_SEV(logger, notification) << "AUTO EXPOSURE   :  NO";
            BOOST_LOG_SEV(logger, notification) << "EXPOSURE TIME   :  " << mDevice->getDayExposureTime();
            BOOST_LOG_SEV(logger, notification) << "GAIN            :  " << mDevice->getDayGain();

            if(!mDevice->setCameraDayExposureTime())
                return false;

            if(!mDevice->setCameraDayGain())
                return false;

    } else {

        BOOST_LOG_SEV(logger, notification) << "DAYTIME         :  NO";
        BOOST_LOG_SEV(logger, notification) << "AUTO EXPOSURE   :  YES";
        BOOST_LOG_SEV(logger, notification) << "EXPOSURE TIME   :  Minimum (" << mDevice->mMinExposureTime << ")"<< mDevice->getNightExposureTime();
        BOOST_LOG_SEV(logger, notification) << "GAIN            :  Minimum (" << mDevice->mMinGain << ")";
        std::cout << "*************************** SET NIGHT AUTO SI" << std::endl;
        if(!mDevice->setCameraExposureTime(mDevice->mMinExposureTime))
            return false;

        if(!mDevice->setCameraGain(mDevice->mMinGain))
            return false;

    }

    // SET FPS.
    if(!mDevice->setCameraFPS())
        return false;

    // INIT CAMERA.
    if(!mDevice->initializeCamera())
        return false;

    // START CAMERA.
    if(!mDevice->startCamera())
        return false;

    return true;

}
