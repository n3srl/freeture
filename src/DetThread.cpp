/*
                                DetThread.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                       2014-2018 Chiara Marmo
*                            GEOPS-UPSUD
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
*   Last modified:      12/03/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    DetThread.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    12/03/2018
* \brief   Detection thread.
*/
#include "DetThread.h"

#include <iostream>

#include <boost/filesystem.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "NodeExporterMetrics.h"
#include "DetectionTemporal.h"
#include "DetectionTemplate.h"
#include "SMTPClient.h"
#include "Stack.h"
#include "Fits3D.h"
#include "Fits2D.h"
#include "TimeDate.h"
#include "CfgParam.h"
#include "EParser.h"

namespace fs = boost::filesystem;
namespace pt = boost::posix_time;

using namespace freeture;
using namespace std;

DetThread::DetThread(boost::circular_buffer<std::shared_ptr<Frame>> fb,
    boost::mutex* fb_m,
    boost::condition_variable* fb_c,
    bool* dSignal,
    boost::mutex* dSignal_m,
    boost::condition_variable* dSignal_c,
    std::shared_ptr<CfgParam> cfg
):
                        pDetMthd(NULL), mForceToReset(false), mMustStop(false),
                        mEventPath(""), mIsRunning(false), mNbDetection(0), mWaitFramesToCompleteEvent(false), mCurrentDataSetLocation(""),
                        mNbWaitFrames(0), mInterruptionStatus(false) {

    frameBuffer = fb;
    frameBuffer_mutex = fb_m;
    frameBuffer_condition = fb_c;
    detSignal = dSignal;
    detSignal_mutex = dSignal_m;
    detSignal_condition = dSignal_c;
    pThread = NULL;

    mFormat = cfg->getCamParam().ACQ_FORMAT;


    mStationName = cfg->getStationParam().STATION_NAME;
    mdp = cfg->getDataParam();
    mdtp = cfg->getDetParam();
    mmp = cfg->getMailParam();
    mfkp = cfg->getFitskeysParam();
    mstp = cfg->getStationParam();

    mNbFramesAround = 0;

    mFitsHeader.loadKeys(mfkp, mstp);

    switch(mdtp.DET_METHOD){

        case TEMPORAL_MTHD :

            {

                pDetMthd = new DetectionTemporal(mdtp, mFormat);

            }

            break;

        case TEMPLATE_MTHD:

            {

                pDetMthd = new DetectionTemplate(mdtp, mFormat);

            }

            break;

    }

}

DetThread::~DetThread(void){

    if(pDetMthd != NULL){

        LOG_INFO << "Remove pDetMthd instance.";
        delete pDetMthd;

    }

    if (pThread!=NULL){

        LOG_INFO<< "Remove detThread instance.";
        delete pThread;

    }
}

bool DetThread::startThread(){

    LOG_INFO<< "Creating detThread...";
    pThread = new boost::thread(boost::ref(*this));

    return true;
}

void DetThread::stopThread(){

    LOG_DEBUG << "DetThread::stopThread";

    // Signal the thread to stop (thread-safe)
    mMustStopMutex.lock();
    mMustStop=true;
    mMustStopMutex.unlock();

    // Wait for the thread to finish.
    while(pThread->timed_join(pt::seconds(2)) == false){

        LOG_INFO << "Interrupting detThread...";
        pThread->interrupt();
    }
}

Detection* DetThread::getDetMethod(){

    return pDetMthd;

}

void DetThread::interruptThread(){

    mInterruptionStatusMutex.lock();
    mInterruptionStatus = true;
    mInterruptionStatusMutex.unlock();

}

void DetThread::operator ()()
{
    m_ThreadID = std::this_thread::get_id();
    Logger::GetLogger()->setLogThread(LogThread::DETECTION_THRED, m_ThreadID,true);

    mIsRunning = true;
    bool stopThread = false;
   
    // Flag to indicate that an event must be complete with more frames.
    bool eventToComplete = false;
    // Reference date to count time to complete an event.
    string refDate;

    LOG_INFO << "\n";
    LOG_INFO << "==============================================";
    LOG_INFO << "=========== Start detection thread ===========";
    LOG_INFO << "==============================================";
    string errorWhen = "NO";
    /// Thread loop.
    try{

        do{
            double t = 0.0;
            try {
                
                /// Wait new frame from AcqThread.
                boost::mutex::scoped_lock lock(*detSignal_mutex);
                while (!(*detSignal)) detSignal_condition->wait(lock);
                *detSignal = false;
                lock.unlock();

                // Check interruption signal from AcqThread.
                mForceToReset = false;
                mInterruptionStatusMutex.lock();
                if(mInterruptionStatus) {
                    LOG_INFO<< "Interruption status : " << mInterruptionStatus;
                    LOG_INFO<< "-> reset forced on detection method.";
                    mForceToReset = true;
                }
                mInterruptionStatusMutex.unlock();
                if(!mForceToReset)
                {
                    errorWhen = "!Force to reset";
                    // Fetch the last grabbed frame.
                    shared_ptr<Frame> lastFrame = make_shared<Frame>();
                    boost::mutex::scoped_lock lock2(*frameBuffer_mutex);
                    if(frameBuffer.size() > 2) 
                        lastFrame = frameBuffer.back();
                    
                    lock2.unlock();

                    t = (double)cv::getTickCount();

                    if (lastFrame->Image)
                    if(lastFrame->Image->data)
                    {
                        mFormat = lastFrame->mFormat;

                        

                        // Run detection process.
                        try {
                            if(pDetMthd->runDetection(lastFrame) && !eventToComplete){

                            // Event detected.
                            LOG_INFO<< "Event detected ! Waiting frames to complete the event..." << endl;
                            eventToComplete = true;

                            // Get a reference date.
                            string currDate = to_simple_string(pt::microsec_clock::universal_time());
                            refDate = currDate.substr(0, currDate.find("."));

                            mNbDetection++;

                        }
                        } catch (exception& e)
                        {
                            LOG_INFO<< "Frame is not valid" << e.what();
                           
                        }
                        

                        // Wait frames to complete the detection.
                        if(eventToComplete){

                            string currDate = to_simple_string(pt::microsec_clock::universal_time());
                            string nowDate = currDate.substr(0, currDate.find("."));
                            pt::ptime t1(pt::time_from_string(refDate));
                            pt::ptime t2(pt::time_from_string(nowDate));
                            pt::time_duration td = t2 - t1;

                            if(td.total_seconds() > mdtp.DET_TIME_AROUND) {

                                LOG_INFO<< "Event completed." << endl;

                                // Build event directory.
                                mEventDate = pDetMthd->getEventDate();
                                LOG_INFO<< "Building event directory..." << endl;

                                if(buildEventDataDirectory())
                                    LOG_INFO<< "Success to build event directory." << endl;
                                else
                                   LOG_ERROR << "Fail to build event directory." << endl;

                                // Save event.
                                LOG_INFO<< "Saving event..." << endl;
                                pDetMthd->saveDetectionInfos(mEventPath, mNbFramesAround);
                                boost::mutex::scoped_lock lock(*frameBuffer_mutex);
                                if(!saveEventData(pDetMthd->getEventFirstFrameNb(), pDetMthd->getEventLastFrameNb()))
                                    LOG_ERROR << "Error saving event data.";
                                else
                                    LOG_INFO<< "Success to save event !" << endl;

                                lock.unlock();

                                // Reset detection.
                                LOG_INFO<< "Reset detection process." << endl;
                                pDetMthd->resetDetection(false);
                                eventToComplete = false;
                                mNbFramesAround = 0;

                            }

                            mNbFramesAround++;
                        }
                    }

                    t = (((double)cv::getTickCount() - t)/ cv::getTickFrequency())*1000;
                    if (LOG_SPAM_FRAME_STATUS)
                    {
                        LOG_DEBUG << " [ TIME DET ] : " << setprecision(3) << fixed << t << " ms " << endl;
                    }
                }else{
                    errorWhen = "Force to reset";
                    // reset method
                    if(pDetMthd != NULL)
                        pDetMthd->resetDetection(false);

                    eventToComplete = false;
                    mNbWaitFrames = 0;

                    mInterruptionStatusMutex.lock();
                    mInterruptionStatus = false;
                    mInterruptionStatusMutex.unlock();

                }

            }catch(const boost::thread_interrupted&){

                LOG_WARNING << "Detection Thread INTERRUPTED";

            } catch(exception& e)
            {
                LOG_ERROR << errorWhen << " An error occured in internal try. See log for details.";
                LOG_ERROR << e.what();
            }

            mMustStopMutex.lock();
            stopThread = mMustStop;
            mMustStopMutex.unlock();

            NodeExporterMetrics::GetInstance().UpdateMetrics(mNbDetection,t);
            NodeExporterMetrics::GetInstance().WriteMetrics();
        }while(!stopThread);

        if(mDetectionResults.size() == 0) {

            LOG_INFO << "-----------------------------------------------" << endl;
            LOG_INFO << "------------->> DETECTED EVENTS : " << mNbDetection << endl;
            LOG_INFO << "-----------------------------------------------" << endl;

        }else {

            // Create Report for videos and frames in input.
            std::ofstream report;
            string reportPath = mdp.DATA_PATH + "detections_report.txt";
            report.open(reportPath.c_str());

            LOG_INFO << "--------------- DETECTION REPORT --------------" << endl;

            for(int i = 0; i < mDetectionResults.size(); i++) {
                report << mDetectionResults.at(i).first << "------> " << mDetectionResults.at(i).second << "\n";
                cout << "- DATASET " << i << " : ";

                if(mDetectionResults.at(i).second > 1)
                    cout << mDetectionResults.at(i).second << " events" << endl;
                else
                    cout << mDetectionResults.at(i).second << " event" << endl;
            }

            LOG_INFO << "-----------------------------------------------" << endl;

            report.close();

        }

    }catch(const char * msg){
        LOG_ERROR << msg;
    }
    catch (exception& e) {
        LOG_ERROR << e.what();
    }
    catch (...) {
        LOG_ERROR << "Generic error occured";
    }

    mIsRunning = false;

    LOG_INFO << "DetThread ended.";
}

bool DetThread::getRunStatus()
{
    return mIsRunning;
}

bool DetThread::buildEventDataDirectory(){
    cout << "DetThrad buildEventDataDirectory" << endl;
    namespace fs =fs;

    // eventDate is the date of the first frame attached to the event.
    string YYYYMMDD = TimeDate::getYYYYMMDD(mEventDate);

    // Data location.
    path p(mdp.DATA_PATH);

    // Create data directory for the current day.
    string fp = mdp.DATA_PATH + mStationName + "_" + YYYYMMDD +"/";
    path p0(fp);

    // Events directory.
    string fp1 = "events/";
    path p1(fp + fp1);

    // Current event directory with the format : STATION_AAAAMMDDThhmmss_UT
    string fp2 = mStationName + "_" + TimeDate::getYYYYMMDDThhmmss(mEventDate) + "_UT/";
    path p2(fp + fp1 + fp2);

    // Final path used by an other function to save event data.
    mEventPath = fp + fp1 + fp2;

    // Check if data path specified in the configuration file exists.
    if(fs::exists(p)){

        // Check DataLocation/STATION_AAMMDD/
        if(fs::exists(p0)){

            // Check DataLocation/STATION_AAMMDD/events/
            if(fs::exists(p1)){

                // Check DataLocation/STATION_AAMMDD/events/STATION_AAAAMMDDThhmmss_UT/
                if(!fs::exists(p2)){

                    // Create DataLocation/STATION_AAMMDD/events/STATION_AAAAMMDDThhmmss_UT/
                    if(!fs::create_directory(p2)){

                        LOG_ERROR << "Fail to create : " << p2;
                        return false;

                    }else{

                        LOG_INFO << "Success to create : " << p2;
                        return true;
                    }

                }

            }else{

                // Create DataLocation/STATION_AAMMDD/events/
                if(!fs::create_directory(p1)){

                    LOG_ERROR << "Fail to create : " << p1;
                    return false;

                }else{

                    // Create DataLocation/STATION_AAMMDD/events/STATION_AAAAMMDDThhmmss_UT/
                    if(!fs::create_directory(p2)){

                        LOG_ERROR << "Fail to create : " << p2;
                        return false;

                    }else{

                        LOG_INFO << "Success to create : " << p2;
                        return true;

                    }
                }
            }

        }else{

            // Create DataLocation/STATION_AAMMDD/
            if(!fs::create_directory(p0)){

                LOG_ERROR << "Fail to create : " << p0;
                return false;

            }else{

                // Create DataLocation/STATION_AAMMDD/events/
                if(!fs::create_directory(p1)){

                    LOG_ERROR << "Fail to create : " << p1;
                    return false;

                }else{

                    // Create DataLocation/STATION_AAMMDD/events/STATION_AAAAMMDDThhmmss_UT/
                    if(!fs::create_directory(p2)){

                        LOG_ERROR << "Fail to create : " << p2;
                        return false;

                    }else{

                        LOG_INFO << "Success to create : " << p2;
                        return true;

                    }
                }
            }
        }

    }else{

        // Create DataLocation/
        if(!fs::create_directory(p)){

            LOG_ERROR << "Fail to create : " << p;
            return false;

        }else{

            // Create DataLocation/STATION_AAMMDD/
            if(!fs::create_directory(p0)){

                LOG_ERROR << "Fail to create : " << p0;
                return false;

            }else{

                //Create DataLocation/STATION_AAMMDD/events/
                if(!fs::create_directory(p1)){

                    LOG_ERROR << "Fail to create : " << p1;
                    return false;

                }else{

                    // Create DataLocation/STATION_AAMMDD/events/STATION_AAAAMMDDThhmmss_UT/
                    if(!fs::create_directory(p2)){

                        LOG_ERROR << "Fail to create : " << p2;
                        return false;

                    }else{

                        LOG_INFO << "Success to create : " << p1;
                        return true;

                    }
                }
            }
        }
    }

    return true;
}

bool DetThread::saveEventData(int firstEvPosInFB, int lastEvPosInFB){
    cout << "DetThread SAVE EVENT DATA" << endl;
    namespace fs =fs;

    // List of data path to attach to the mail notification.
    vector<string> mailAttachments;

    // Number of the first frame to save. It depends of how many frames we want to keep before the event.
    int numFirstFrameToSave = firstEvPosInFB - mNbFramesAround;

    // Number of the last frame to save. It depends of how many frames we want to keep after the event.
    int numLastFrameToSave = lastEvPosInFB + mNbFramesAround;

    // If the number of the first frame to save for the event is not in the framebuffer.
    // The first frame to save become the first frame available in the framebuffer.
    if(frameBuffer.front()->mFrameNumber > numFirstFrameToSave)
        numFirstFrameToSave = frameBuffer.front()->mFrameNumber;

    // Check the number of the last frame to save.
    if(frameBuffer.back()->mFrameNumber < numLastFrameToSave)
        numLastFrameToSave = frameBuffer.back()->mFrameNumber;

    // Total frames to save.
    int nbTotalFramesToSave = numLastFrameToSave - numFirstFrameToSave;

    // Count number of digit on nbTotalFramesToSave.
    int n = nbTotalFramesToSave;
    int nbDigitOnNbTotalFramesToSave = 0;

    while(n!=0){
        n/=10;
        ++nbDigitOnNbTotalFramesToSave;
    }

    LOG_INFO << "> First frame to save  : " << numFirstFrameToSave;
    LOG_INFO << "> Lst frame to save    : " << numLastFrameToSave;
    LOG_INFO << "> First event frame    : " << firstEvPosInFB;
    LOG_INFO << "> Last event frame     : " << lastEvPosInFB;
    LOG_INFO << "> Frames before        : " << mNbFramesAround;
    LOG_INFO << "> Frames after         : " << mNbFramesAround;
    LOG_INFO << "> Total frames to save : " << nbTotalFramesToSave;
    LOG_INFO << "> Total digit          : " << nbDigitOnNbTotalFramesToSave;

    TimeDate::Date dateFirstFrame;

    int c = 0;

    // Init video avi
    cv::VideoWriter *video = NULL;

    if(mdtp.DET_SAVE_AVI) {
        video = new cv::VideoWriter(mEventPath + "video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5, cv::Size(static_cast<int>(frameBuffer.front()->Image->cols), static_cast<int>(frameBuffer.front()->Image->rows)), false);
    }

    // Init fits 3D.
    Fits3D fits3d;

    if(mdtp.DET_SAVE_FITS3D) {

        fits3d = Fits3D(mFormat, frameBuffer.front()->Image->rows, frameBuffer.front()->Image->cols, (numLastFrameToSave - numFirstFrameToSave +1), mEventPath + "fits3D");
        pt::ptime time = pt::microsec_clock::universal_time();
        fits3d.kDATE = pt::to_iso_extended_string(time);

        // Name of the fits file.
        fits3d.kFILENAME = mEventPath + "fitscube.fit";

    }

    // Init sum.
    Stack stack = Stack(mdp.FITS_COMPRESSION_METHOD, mfkp, mstp);

    // Exposure time sum.
    double sumExpTime = 0.0;
    double firstExpTime = 0.0;
    bool varExpTime = false;

    // Loop framebuffer.
    boost::circular_buffer<shared_ptr<Frame>>::iterator it;
    for (it = frameBuffer.begin(); it != frameBuffer.end(); ++it) {

        // Get infos about the first frame of the event for fits 3D.
        if ((*it)->mFrameNumber == numFirstFrameToSave && mdtp.DET_SAVE_FITS3D) {

            fits3d.kDATEOBS = TimeDate::getIsoExtendedFormatDate((*it)->mDate);

            // Gain.
            fits3d.kGAINDB = (*it)->mGain;
            // Saturation.
            fits3d.kSATURATE = (*it)->mSaturatedValue;
            // FPS.
            fits3d.kCD3_3 = (*it)->mFps;
            // CRVAL1 : sideral time.
            double  julianDate = TimeDate::gregorianToJulian((*it)->mDate);
            double  julianCentury = TimeDate::julianCentury(julianDate);
            double  sideralT = TimeDate::localSideralTime_2(julianCentury, (*it)->mDate.hours, (*it)->mDate.minutes, (int)(*it)->mDate.seconds, mFitsHeader.kSITELONG);
            fits3d.kCRVAL1 = sideralT;
            // Projection and reference system
            fits3d.kCTYPE1 = "RA---ARC";
            fits3d.kCTYPE2 = "DEC--ARC";
            // Equinox
            fits3d.kEQUINOX = 2000.0;
            firstExpTime = (*it)->mExposure;
            dateFirstFrame = (*it)->mDate;

        }

        // Get infos about the last frame of the event record for fits 3D.
        if ((*it)->mFrameNumber == numLastFrameToSave && mdtp.DET_SAVE_FITS3D) {
            cout << "DATE first : " << dateFirstFrame.hours << " H " << dateFirstFrame.minutes << " M " << dateFirstFrame.seconds << " S" << endl;
            cout << "DATE last : " << (*it)->mDate.hours << " H " << (*it)->mDate.minutes << " M " << (*it)->mDate.seconds << " S" << endl;
            fits3d.kELAPTIME = ((*it)->mDate.hours * 3600 + (*it)->mDate.minutes * 60 + (*it)->mDate.seconds) - (dateFirstFrame.hours * 3600 + dateFirstFrame.minutes * 60 + dateFirstFrame.seconds);

        }

        // If the current frame read from the framebuffer has to be saved.
        if ((*it)->mFrameNumber >= numFirstFrameToSave && (*it)->mFrameNumber < numLastFrameToSave) {

            // Save fits2D.
            if (mdtp.DET_SAVE_FITS2D) {

                string fits2DPath = mEventPath + "fits2D/";
                string fits2DName = "frame_" + Conversion::numbering(nbDigitOnNbTotalFramesToSave, c) + Conversion::intToString(c);
                LOG_INFO << ">> Saving fits2D : " << fits2DName;

                Fits2D newFits(fits2DPath);
                newFits.loadKeys(mfkp, mstp);
                // Frame's acquisition date.
                newFits.kDATEOBS = TimeDate::getIsoExtendedFormatDate((*it)->mDate);
                // Fits file creation date.
                boost::posix_time::ptime time = pt::second_clock::universal_time();
                // YYYYMMDDTHHMMSS,fffffffff where T is the date-time separator
                newFits.kDATE = pt::to_iso_string(time);
                // Name of the fits file.
                newFits.kFILENAME = fits2DName;
                // Exposure time.
                newFits.kONTIME = (*it)->mExposure / 1000000.0;
                // Gain.
                newFits.kGAINDB = (*it)->mGain;
                // Saturation.
                newFits.kSATURATE = (*it)->mSaturatedValue;
                // FPS.
                newFits.kCD3_3 = (*it)->mFps;
                // CRVAL1 : sideral time.
                double  julianDate = TimeDate::gregorianToJulian((*it)->mDate);
                double  julianCentury = TimeDate::julianCentury(julianDate);
                double  sideralT = TimeDate::localSideralTime_2(julianCentury, (*it)->mDate.hours, (*it)->mDate.minutes, (*it)->mDate.seconds, mFitsHeader.kSITELONG);
                newFits.kCRVAL1 = sideralT;
                newFits.kEXPOSURE = (*it)->mExposure / 1000000.0;
                // Projection and reference system
                newFits.kCTYPE1 = "RA---ARC";
                newFits.kCTYPE2 = "DEC--ARC";
                // Equinox
                newFits.kEQUINOX = 2000.0;

                if (!fs::exists(path(fits2DPath))) {
                    if (fs::create_directory(path(fits2DPath)))
                        LOG_INFO << "Success to create directory : " << fits2DPath;
                }

                switch (mFormat) {

                case CamPixFmt::MONO12:
                {
                    newFits.writeFits((*it)->Image, S16, fits2DName, mdp.FITS_COMPRESSION_METHOD);
                }
                break;

                default:

                {
                    newFits.writeFits((*it)->Image, UC8, fits2DName, mdp.FITS_COMPRESSION_METHOD);
                }
                }
            }

            if (mdtp.DET_SAVE_AVI) {
                cv::Mat iv = Conversion::convertTo8UC1(*(*it)->Image.get());
                if (video->isOpened()) {
                    video->write(iv);
                }
            }

            // Add a frame to fits cube.
            if (mdtp.DET_SAVE_FITS3D) {

                if (firstExpTime != (*it)->mExposure)
                    varExpTime = true;

                sumExpTime += (*it)->mExposure;
                fits3d.addImageToFits3D(*(*it)->Image.get());

            }

            // Add frame to the event's stack.
            if (mdtp.DET_SAVE_SUM && (*it)->mFrameNumber >= firstEvPosInFB && (*it)->mFrameNumber <= lastEvPosInFB) {

                stack.addFrame((*it));

            }

            c++;

        }
    }

    if(mdtp.DET_SAVE_AVI) {
        if(video != NULL)
            delete video;
    }

    // ********************************* SAVE EVENT IN FITS CUBE  ***********************************

    if(mdtp.DET_SAVE_FITS3D) {

        // Exposure time of a single frame.
        if(varExpTime)
            fits3d.kEXPOSURE = 999999;
        else {
            it = frameBuffer.begin();
            fits3d.kEXPOSURE = (*it)->mExposure/1000000.0;
        }

        // Exposure time sum of frames in the fits cube.
        fits3d.kONTIME = sumExpTime/1000000.0;

        fits3d.writeFits3D();

    }

    // ********************************* SAVE EVENT STACK IN FITS  **********************************

    if(mdtp.DET_SAVE_SUM) {

        stack.saveStack(mEventPath, mdtp.DET_SUM_MTHD, mdtp.DET_SUM_REDUCTION);

    }

    // ************************** EVENT STACK WITH HISTOGRAM EQUALIZATION ***************************

    if(mdtp.DET_SAVE_SUM_WITH_HIST_EQUALIZATION) {

        shared_ptr<cv::Mat> s,s1, eqHist;

        float bzero  = 0.0;
        float bscale = 1.0;
        s = stack.reductionByFactorDivision(bzero,bscale);
        EParser<CamPixFmt> fmt;

        string value = fmt.getStringEnum(mFormat);

        cout << "mFormat : " << value << endl;
        cv::Mat& mat = *s.get();

        if (mFormat != CamPixFmt::MONO8) {
            Conversion::convertTo8UC1(mat).copyTo(mat);
        }

        equalizeHist(mat, *eqHist.get());
        SaveImg::saveJPEG(*eqHist.get(),mEventPath+mStationName+"_"+TimeDate::getYYYYMMDDThhmmss(mEventDate)+"_UT");
    }

    // *********************************** SEND MAIL NOTIFICATION ***********************************
    LOG_INFO << "Prepare mail..." << mmp.MAIL_DETECTION_ENABLED;
    if(mmp.MAIL_DETECTION_ENABLED) {

        LOG_INFO << "Sending mail...";

        for(int i = 0; i < pDetMthd->getDebugFiles().size(); i++) {

            if(fs::exists( mEventPath + pDetMthd->getDebugFiles().at(i))) {

                LOG_INFO << "Send : " << mEventPath << pDetMthd->getDebugFiles().at(i);
                mailAttachments.push_back(mEventPath + pDetMthd->getDebugFiles().at(i));

            }
        }

        if(mdtp.DET_SAVE_SUM_WITH_HIST_EQUALIZATION &&fs::exists(mEventPath + mStationName + "_" + TimeDate::getYYYYMMDDThhmmss(mEventDate) + "_UT.jpg")) {

            LOG_INFO << "Send : " << mEventPath << mStationName << "_" << TimeDate::getYYYYMMDDThhmmss(mEventDate) << "_UT.jpg";
            mailAttachments.push_back(mEventPath + mStationName + "_" + TimeDate::getYYYYMMDDThhmmss(mEventDate) + "_UT.jpg");

        }

        SMTPClient::sendMail(   mmp.MAIL_SMTP_SERVER,
                                mmp.MAIL_SMTP_LOGIN,
                                mmp.MAIL_SMTP_PASSWORD,
                                "freeture@" + mStationName +".fr",
                                mmp.MAIL_RECIPIENTS,
                                mStationName  + "-" + TimeDate::getYYYYMMDDThhmmss(mEventDate),
                                mStationName + "\n" + mEventPath,
                                mailAttachments,
                                mmp.MAIL_CONNECTION_TYPE);

    }

    return true;

}
