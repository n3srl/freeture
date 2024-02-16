/*
                                Device.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                       2018 Chiara Marmo
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
*   Last modified:      19/03/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    Device.cpp
* \author  Yoan Audureau -- Chiara Marmo -- GEOPS-UPSUD
* \version 1.2
* \date    19/03/2018
* \brief
*/
#include "Device.h"

#include "Camera.h"

#include "Logger.h"
#include "EParser.h"
#include "ECamSdkType.h"
#include "CameraDeviceManager.h"
#include "CameraFrames.h"
#include "CameraVideo.h"

#ifdef USE_ARENA
#include "CameraLucidArena_PHX016S.h"
#endif

#ifdef LINUX
#include "CameraV4l2.h"
#include "CameraGigeAravis.h"
#include "CameraLucidArena.h"
#endif

using namespace std;
using namespace freeture;

namespace fs = boost::filesystem;

/// <summary>
/// Apply parameters to this device runtime instance - NO APPLY TO DEVICE JUST MEMORY
/// </summary>
/// <param name="cp"></param>
/// <param name="fp"></param>
/// <param name="vp"></param>
/// <param name="cid"></param>
void Device::Setup(cameraParam cp, framesParam fp, videoParam vp)
{
    LOG_DEBUG << "Device::Setup";
    LOG_DEBUG << "PARAM NIGHT EXP " << cp.ACQ_NIGHT_EXPOSURE << endl;

    /* mCam        = NULL;
    mCamID      = 0;
    mVerbose    = true;
    mNbDev      = -1;
    mVideoFramesInput = false;
    mGenCamID = cid; */
    mFPS = cp.ACQ_FPS;
    mNightExposure = cp.ACQ_NIGHT_EXPOSURE;
    mNightGain = cp.ACQ_NIGHT_GAIN;
    mDayExposure = cp.ACQ_DAY_EXPOSURE;
    mDayGain = cp.ACQ_DAY_GAIN;

    //mShiftBits = cp.SHIFT_BITS;
    mFormat = cp.ACQ_FORMAT;
    mCustomSize = cp.ACQ_RES_CUSTOM_SIZE;
    mStartX = cp.ACQ_STARTX;
    mStartY = cp.ACQ_STARTY;
    mSizeWidth = cp.ACQ_WIDTH;
    mSizeHeight = cp.ACQ_HEIGHT;
    mDeviceType = UNDEFINED_INPUT_TYPE;
    m_VideoParam = vp;
    m_FramesParam = fp;
    //mNbFrame = 0;

    minExposureTime = -1;
    maxExposureTime = -1;
    minGain = -1;
    maxGain = -1;
}


Device::Device() {
    LOG_DEBUG << "Device::Device";

    mFormat         = MONO8;
    mNightExposure  = 0;
    mNightGain      = 0;
    mDayExposure    = 0;
    mDayGain        = 0;
    minExposureTime = -1;
    maxExposureTime = -1;
    minGain = -1;
    maxGain = -1;
    mFPS            = 30;
    mCustomSize     = false;
    mStartX         = 0;
    mStartY         = 0;
    mSizeWidth      = 1440;
    mSizeHeight     = 1080;
    mCam            = nullptr;
    mVideoFramesInput = false;
    //mShiftBits      = false;
    mVerbose        = true;
    mDeviceType     = UNDEFINED_INPUT_TYPE;
    vector<string> finput, vinput;
    m_VideoParam.INPUT_VIDEO_PATH = vinput;
    m_FramesParam.INPUT_FRAMES_DIRECTORY_PATH = finput;
    //mNbFrame = 0;

}

Device::~Device()
{
    LOG_DEBUG <<  "Device::~Device";

    if(mCam != nullptr)
    {
        LOG_DEBUG <<  "Device::~Device: deallocating camera";
        delete mCam;
    }

}

bool Device::firstIinitializeCamera(string m_ConfigurationFilePath)
{
    LOG_DEBUG << "Device:firstIinitializeCamera(\"" << m_ConfigurationFilePath << "\")" << endl;
    
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    Device* m_Device = m_CameraDeviceManager->getDevice();
    if(m_Device != NULL)
    {
        return m_Device->mCam->FirstInitializeCamera(m_ConfigurationFilePath);
    }
    return false;
}

void Device::setVerbose(bool status) {
    LOG_DEBUG << "Device::setVerbose";
    mVerbose = status;

}

bool Device::getDeviceName() {
    LOG_DEBUG << "Device::getDeviceName";

    return mCam->getCameraName();

}

bool Device::setCameraPixelFormat() {
    LOG_DEBUG << "Device::setCameraPixelFormat";
    if(!mCam->setPixelFormat(mFormat)){
        mCam->grabCleanse();
        
        LOG_ERROR << "Fail to set camera format.";
        return false;
    }

    return true;
}

bool Device::getSupportedPixelFormats() {
    LOG_DEBUG << "Device::getSupportedPixelFormats";


    mCam->getAvailablePixelFormats();
    return true;

}

InputDeviceType Device::getDeviceType() {
    LOG_DEBUG << "Device::getDeviceType";

    return mCam->getDeviceType();

}

bool Device::getCameraExposureBounds(double &min, double &max) {
    LOG_DEBUG << "Device::getCameraExposureBounds(double,double)";

    mCam->getExposureBounds(min, max);
    return true;
}

void Device::getCameraExposureBounds() {
    LOG_DEBUG << "Device::getCameraExposureBounds";

    mCam->getExposureBounds(minExposureTime, maxExposureTime);

}

bool Device::getCameraFPSBounds(double &min, double &max) {
    LOG_DEBUG << "Device::getCameraFPSBounds(double,double)";
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();

    m_CameraDeviceManager->getDevice()->mCam->getFPSBounds(min, max);
    

    return true;
}

void Device::getCameraFPSBounds() {
    LOG_DEBUG << "Device::getCameraFPSBounds";
    mCam->getFPSBounds(minFPS, maxFPS);
}


bool Device::getCameraGainBounds(double &min, double &max) {
    LOG_DEBUG << "Device::getCameraGainBounds(double,double)";


    mCam->getGainBounds(min, max);
    return true;
}

void Device::getCameraGainBounds() {
    LOG_DEBUG << "Device::getCameraGainBounds";

    mCam->getGainBounds(minGain, maxGain);
}

bool Device::setCameraNightExposureTime()
{   
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();

    Device* m_Device = m_CameraDeviceManager->getDevice();
    
    if(!mCam->setExposureTime(mNightExposure)) {
        LOG_ERROR << "Fail to set night exposure time to " << mNightExposure;
        mCam->grabCleanse();
        return false;
    }

    getCameraFPSBounds();

    return true;
}

bool Device::setCameraDayExposureTime() {
    LOG_DEBUG << "Device::setCameraDayExposureTime";

    if(!mCam->setExposureTime(mDayExposure)) {
    
        LOG_ERROR << "Fail to set day exposure time to " << mDayExposure;
        mCam->grabCleanse();
        return false;
    }

    getCameraFPSBounds();

    return true;
}

bool Device::setCameraNightGain() {
    LOG_DEBUG << "Device::setCameraNightGain "<<mNightGain;

    if(!mCam->setGain(mNightGain)) {
        LOG_ERROR << "Fail to set night gain to " << mNightGain;
        mCam->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraDayGain() {
    LOG_DEBUG <<  "Device::setCameraDayGain";

    if(!mCam->setGain(mDayGain)) {
        LOG_ERROR << "Fail to set day gain to " << mDayGain;
        mCam->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraFPS(double value) {
    LOG_DEBUG << "Device::setCameraFPS ["<< value << "]";

    if(!mCam->setFPS(value)) {
        LOG_ERROR << "Fail to set fps to " << value;
        mCam->grabCleanse();
        return false;
    }

    return true;
}


bool Device::setCameraExposureTime(double value) {
    LOG_DEBUG << "Device::setCameraExposureTime";

    if(!mCam->setExposureTime(value)) {
        LOG_ERROR << "Fail to set exposure time to " << value;
        mCam->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraGain(double value) {
    LOG_DEBUG <<  "Device::setCameraGain";

    if(!mCam->setGain(value)) {
        LOG_ERROR << "Fail to set gain to " << value;
        mCam->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraAutoExposure(bool value)
{
    LOG_DEBUG << "Device::setCameraAutoExposure";
    //arv_camera_set_exposure_time_auto(camera, &error)
    if(!mCam->setAutoExposure(value)) {
        LOG_ERROR << "Fail to set gain to " << value;
        mCam->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraFPS() {
    LOG_DEBUG << "Device::setCameraFPS";

    if(!mCam->setFPS(mFPS)) {
        LOG_ERROR << "Fail to set FPS to " << mFPS;
        mCam->grabCleanse();
        return false;
    }

    return true;
}

bool Device::initializeCamera() {
    LOG_DEBUG << "Device::initializeCamera";

    if(!mCam->grabInitialization()){
        LOG_ERROR << "Fail to initialize camera.";
        mCam->grabCleanse();
        return false;
    }

    return true;
}

bool Device::startCamera() {
    LOG_DEBUG << "Device::startCamera";

    LOG_INFO << "Starting camera...";
    if(!mCam->acqStart())
        return false;

    return true;
}

bool Device::stopCamera()
{
    LOG_DEBUG << "Device::stopCamera" ;
    LOG_INFO << "Stopping camera...";
    mCam->acqStop();
    mCam->grabCleanse();

    return true;
}

/*
 * Called every frame
 */
bool Device::runContinuousCapture(Frame &img)
{
    //cout << "Device::runContinuousCapture" << endl;

    try
    {
        if(mCam == nullptr) 
            LOG_ERROR << "MCAM IS NULL" << endl;
        
        if(mCam->grabImage(img)) {
            //img.mFrameNumber = mNbFrame;
            //mNbFrame++;
            return true;
        }
    }
    catch (exception& ex)
    {
        LOG_ERROR << "Exception grabImage..." << ex.what();
    }
    
    return false;
}

bool Device::runSingleCapture(Frame &img) {
    LOG_DEBUG << "Device::runSingleCapture";

    if(mCam->grabSingleImage(img))
        return true;

    return false;

}

bool Device::getDeviceCameraSizeParams(int& x,int& y,int& height,int& width)
{
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    Device* m_Device = m_CameraDeviceManager->getDevice();
    x = m_Device->mStartX;
    y = m_Device->mStartY;
    height = m_Device->mSizeHeight;
    width = m_Device->mSizeWidth;
    return true;
}

bool Device::setCameraSize()
{
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    Device* m_Device = m_CameraDeviceManager->getDevice();
    LOG_DEBUG << "Device::setCameraSize; "<< m_Device->mSizeWidth <<"x"<< m_Device->mSizeHeight ;
    
    Camera* mCam = m_Device->mCam;
    
    if(!mCam->setSize(m_Device->mStartX, m_Device->mStartY, m_Device->mSizeWidth, m_Device->mSizeHeight, m_Device->mCustomSize)) 
    {
        LOG_ERROR << "Device::setCameraSize;" <<"Fail to set camera size.";
        return false;
    }


    return true;
}

bool Device::setCameraSize(int x, int y, int w, int h) {
    LOG_DEBUG << "Device::setCameraSize;"<<w<<"x"<<h;

    if(!mCam->setSize(x, y, w, h, true)) {
       LOG_ERROR << "Device::setCameraSize;" << "Fail to set camera size.";
        return false;
    }

    return true;
}

bool Device::getCameraFPS(double &fps) {
    LOG_DEBUG << "Device::getCameraFPS" ;

    if(!mCam->getFPS(fps)) {
        //BOOST_LOG_SEV(logger, fail) << "Fail to get fps value from camera.";
        return false;
    }

    return true;
}

/*
 * Called every frame
 */
bool Device::getCameraStatus() {
    //cout << "Device::getCameraStatus" << endl;

    return mCam->getStopStatus();
}

bool Device::getCameraDataSetStatus() {
    LOG_DEBUG << "Device::getCameraDataSetStatus";

    return mCam->getDataSetStatus();
}

bool Device::loadNextCameraDataSet(string &location) {
    LOG_DEBUG << "Device::loadNextCameraDataSet";

    return mCam->loadNextDataSet(location);
}

bool Device::getExposureStatus() {
    LOG_DEBUG << "Device::getExposureStatus";

    return mCam->m_ExposureAvailable;
}

bool Device::getGainStatus() {
    LOG_DEBUG << "Device::getGainStatus";

    return mCam->m_GainAvailable;
}

Camera* Device::getCamera()
{
    return mCam;
}
