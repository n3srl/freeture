/*
                                CameraVideo.cpp

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
* \file    CameraVideo.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    13/06/2014
* \brief   Acquisition thread with video in input.
*/

#include "CameraVideo.h"
#include "Logger.h"
#include <boost/date_time.hpp>

using namespace freeture;

CameraVideo::CameraVideo(CameraDescription camera_descriptor, cameraParam settings , std::vector<std::string> videoList, bool verbose) :
    Camera(camera_descriptor, settings),
    mVideoID(0),
    mFrameWidth(0),
    mFrameHeight(0),
    mReadDataStatus(false)
{

    mVideoList = videoList;

    // Open the video file for reading.
    if(mVideoList.size()>0)
        mCap = cv::VideoCapture(videoList.front());
    else
        throw "No video path in input.";

    m_ExposureAvailable = false;
    m_GainAvailable = false;

    m_CameraDescriptor.DeviceType = VIDEO;
}

CameraVideo::~CameraVideo(void){

}

bool CameraVideo::grabInitialization(){

    if(!mCap.isOpened()) {
         LOG_ERROR << "Cannot open the video file";
         return false;
    }

    return true;

}

bool CameraVideo::getStopStatus(){

    return mReadDataStatus;

}

bool CameraVideo::getDataSetStatus(){

    if(mVideoID == mVideoList.size())
        return false;
    else
        return true;
}

bool CameraVideo::loadNextDataSet(std::string &location){

    if(mVideoID != 0){

       LOG_INFO << "Change video : " << mVideoID << " - Path : " << mVideoList.at(mVideoID) << std::endl;

        mCap = cv::VideoCapture(mVideoList.at(mVideoID));

        if(!mCap.isOpened()){

            LOG_ERROR << std::endl;
             return false;

        }else{

            LOG_INFO << std::endl;

        }

        mFrameHeight = mCap.get(cv::CAP_PROP_FRAME_HEIGHT);

        mFrameWidth = mCap.get(cv::CAP_PROP_FRAME_WIDTH);

        mReadDataStatus = false;

    }

    return true;

}

bool CameraVideo::createDevice()
{
    return true;
}

bool CameraVideo::grabImage(Frame &img){

    cv::Mat frame;

    if(mCap.read(frame)) {

        //BGR (3 channels) to G (1 channel)
        cvtColor(frame, frame, cv::COLOR_BGRA2GRAY);

        boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
        Frame f;
        //Frame f = Frame( frame, 0, 0, to_iso_extended_string(time));

        img = f;
        img.mFrameNumber = mCap.get(cv::CAP_PROP_POS_FRAMES);
        img.mFrameRemaining = mCap.get(cv::CAP_PROP_FRAME_COUNT) - mCap .get(cv::CAP_PROP_POS_FRAMES);
        return true;

    }

    if(mCap.get(cv::CAP_PROP_FRAME_COUNT) - mCap .get(cv::CAP_PROP_POS_FRAMES) <=0) {

        mVideoID++;
        mReadDataStatus = true;

    }

    return false;
}

bool CameraVideo::initSDK()
{
    return true;
}

bool CameraVideo::initOnce()
{
    return true;
}

bool CameraVideo::init()
{
    return true;
}

void CameraVideo::fetchBounds(parameters&)
{

}

void CameraVideo::configure(parameters&)
{

}

bool CameraVideo::configurationCheck(parameters&)
{
    return true;
}

double CameraVideo::getMinExposureTime()
{
    return 0.0;
}

bool CameraVideo::destroyDevice()
{
    return false;
}

