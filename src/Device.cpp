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

#include <boost/filesystem.hpp>

#include "Camera.h"

#include "EParser.h"
#include "ECamSdkType.h"
#include "CameraDeviceManager.h"

using namespace std;
using namespace freeture;

namespace fs = boost::filesystem;

void printValues(cameraParam& cp, string log) {
    EParser<CamPixFmt> fmt;
    string val = fmt.getStringEnum(cp.ACQ_FORMAT);

    LOG_DEBUG << log << "ACQ_FPS=" << cp.ACQ_FPS << endl;
    LOG_DEBUG << log << "ACQ_DAY_GAIN=" << cp.ACQ_DAY_GAIN << endl;
    LOG_DEBUG << log << "ACQ_DAY_EXPOSURE=" << cp.ACQ_DAY_EXPOSURE << endl;

    LOG_DEBUG << log << "ACQ_NIGHT_GAIN=" << cp.ACQ_NIGHT_GAIN << endl;
    LOG_DEBUG << log << "ACQ_NIGHT_EXPOSURE=" << cp.ACQ_NIGHT_EXPOSURE << endl;

    LOG_DEBUG << log << "ACQ_FORMAT= " << val << endl;

    LOG_DEBUG << log << "ACQ_RES_CUSTOM_SIZE=" << cp.ACQ_RES_CUSTOM_SIZE << endl;

    LOG_DEBUG << log << "ACQ_WIDTH=" << cp.ACQ_WIDTH << endl;
    LOG_DEBUG << log << "ACQ_HEIGHT=" << cp.ACQ_HEIGHT << endl;

    LOG_DEBUG << log << "ACQ_STARTX=" << cp.ACQ_STARTX << endl;
    LOG_DEBUG << log << "ACQ_STARTY=" << cp.ACQ_STARTY << endl;
}

void printValues(CameraSettings& cp, string log)
{
    EParser<CamPixFmt> fmt;
    string val = fmt.getStringEnum(cp.PixelFormat);

    LOG_DEBUG << log << "PixelFormat= " << val << endl;
    LOG_DEBUG << log << "Gain=" << cp.Gain << endl;
    LOG_DEBUG << log << "FPS=" << cp.FPS << endl;
    LOG_DEBUG << log << "Exposure=" << cp.Exposure << endl;
    LOG_DEBUG << log << "CustomSize=" << cp.CustomSize << endl;
    LOG_DEBUG << log << "SizeWidth=" << cp.SizeWidth << endl;
    LOG_DEBUG << log << "SizeHeight=" << cp.SizeHeight << endl;
    LOG_DEBUG << log << "StartX=" << cp.StartX << endl;
    LOG_DEBUG << log << "StartY=" << cp.StartY << endl;
}





/// <summary>
/// Apply parameters to this device runtime instance - NO APPLY TO DEVICE JUST MEMORY
/// </summary>
/// <param name="cp"></param>
/// <param name="fp"></param>
/// <param name="vp"></param>
/// <param name="cid"></param>
void Device::Setup(cameraParam cp, framesParam fp, videoParam vp)
{

    printValues(cp, "Device::Setup;\t\t\t");

    CameraSetting.FPS = cp.ACQ_FPS;
    CameraSetting.PixelFormat = cp.ACQ_FORMAT;
    CameraSetting.CustomSize = cp.ACQ_RES_CUSTOM_SIZE;
    CameraSetting.StartX = cp.ACQ_STARTX;
    CameraSetting.StartY = cp.ACQ_STARTY;
    CameraSetting.SizeWidth = cp.ACQ_WIDTH;
    CameraSetting.SizeHeight = cp.ACQ_HEIGHT;

    m_VideoParam = vp;
    m_FramesParam = fp;
}



Device::Device() {
    LOG_DEBUG << "Device::Device";
    
    CameraSetting.PixelFormat   = CamPixFmt::MONO8;
    CameraSetting.FPS           = 30;
    CameraSetting.CustomSize    = false;
    CameraSetting.StartX        = 0;
    CameraSetting.StartY        = 0;
    CameraSetting.SizeWidth     = 1440;
    CameraSetting.SizeHeight    = 1080;
    
    printValues(CameraSetting, "Device::Setup;\t\t\t");

    m_Camera            = nullptr;
    mVideoFramesInput = false;
    
    vector<string> finput, vinput;

    m_VideoParam.INPUT_VIDEO_PATH = vinput;
    m_FramesParam.INPUT_FRAMES_DIRECTORY_PATH = finput;
}

Device::~Device()
{
    LOG_DEBUG <<  "Device::~Device";

    if(m_Camera != nullptr)
    {
        LOG_DEBUG <<  "Device::~Device: deallocating camera";
        delete m_Camera;
    }

}

bool Device::firstIinitializeCamera(string m_ConfigurationFilePath)
{
    LOG_DEBUG << "Device:firstIinitializeCamera(\"" << m_ConfigurationFilePath << "\")" << endl;
    
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    Device* m_Device = m_CameraDeviceManager->getDevice();
    if(m_Device != NULL)
    {
        return m_Device->m_Camera->FirstInitializeCamera(m_ConfigurationFilePath);
    }
    return false;
}

string Device::getDeviceModel() {
    LOG_DEBUG << "Device::getDeviceModel";

    return m_Camera->getModel();

}

bool Device::getDeviceName() {
    LOG_DEBUG << "Device::getDeviceName";

    return m_Camera->getCameraName();

}

bool Device::setCameraPixelFormat() {
    LOG_DEBUG << "Device::setCameraPixelFormat";

    if(!m_Camera->setPixelFormat())
    {
        m_Camera->grabCleanse();
        
        LOG_ERROR << "Device::setCameraPixelFormat;" << "Fail to set camera format.";
        return false;
    }

    return true;
}

bool Device::getSupportedPixelFormats()
{
    LOG_DEBUG << "Device::getSupportedPixelFormats";
    m_Camera->getAvailablePixelFormats();
    return true;
}

InputDeviceType Device::getDeviceType() {
    if (LOG_SPAM_FRAME_STATUS)
        LOG_DEBUG << "Device::getDeviceType";
    return m_Camera->getDeviceType();
}

bool Device::setCameraFPS(double value) {
    LOG_DEBUG << "Device::setCameraFPS("<< value << ")";

    if(!m_Camera->setFPS(value)) {
        LOG_ERROR << "Device::setCameraFPS;" << "Fail to set fps to " << value;
        m_Camera->grabCleanse();
        return false;
    }

    return true;
}



bool Device::setCameraExposureTime() {
    LOG_DEBUG << "Device::setCameraExposureTime";

    if (!m_Camera->setExposureTime(CameraSetting.Exposure)) 
    {
        LOG_ERROR << "Fail to set Exposure to " << CameraSetting.Exposure;
        m_Camera->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraExposureTime(double value) {
    LOG_DEBUG << "Device::setCameraExposureTime";

    if(!m_Camera->setExposureTime(value)) {
        LOG_ERROR << "Fail to set exposure time to " << value;
        m_Camera->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraGain(double value) {
    LOG_DEBUG <<  "Device::setCameraGain";

    if(!m_Camera->setGain(value)) {
        LOG_ERROR << "Fail to set gain to " << value;
        m_Camera->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraGain() {
    LOG_DEBUG << "Device::setCameraGain";

    if (!m_Camera->setGain(CameraSetting.Gain)) 
    {
        LOG_ERROR << "Fail to set Gain to " << CameraSetting.Gain;
        m_Camera->grabCleanse();
        return false;
    }

    return true;
}

bool Device::setCameraFPS() {
    LOG_DEBUG << "Device::setCameraFPS";

    if(!m_Camera->setFPS(CameraSetting.FPS)) {
        LOG_ERROR << "Fail to set FPS to " << CameraSetting.FPS;
        m_Camera->grabCleanse();
        return false;
    }

    return true;
}

//apply current configuration to the camera
bool Device::initializeCamera() {

   LOG_DEBUG << "Device::initializeCamera";

    printValues(CameraSetting, "Device::initializeCamera;");

    //Apply configuration to camera
    // SET FPS.
    if (!setCameraFPS())
        return false;

    // SET FPS.
    if (!setCameraGain())
        return false;

    // SET FPS.
    if (!setCameraExposureTime())
        return false;

    //apply streaming rules
    if(!m_Camera->grabInitialization())
    {
        LOG_ERROR << "Fail to initialize camera.";
        m_Camera->grabCleanse();
        return false;
    }

    return true;
}


bool Device::startCamera(EAcquisitionMode mode) {
    LOG_DEBUG << "Device::startCamera";

    LOG_INFO << "Device::startCamera;" <<"Starting camera...";
    switch (mode) {
    case EAcquisitionMode::CONTINUOUS:{
            if (!m_Camera->acqStart())
            return false;
            break;
    }
    case EAcquisitionMode::SCHEDULED:
    case EAcquisitionMode::REGULAR: {
        if (!m_Camera->acqStart(false))
            return false;
            break;
    }
    }
    return true;
}

bool Device::stopCamera()
{
    LOG_DEBUG << "Device::stopCamera" ;
    LOG_INFO << "Stopping camera...";
    m_Camera->acqStop();
    m_Camera->grabCleanse();

    return true;
}

/*
 * Called every frame
 */
bool Device::runContinuousCapture(shared_ptr<Frame> img)
{
    if (LOG_SPAM_FRAME_STATUS)
        LOG_DEBUG << "Device::runContinuousCapture";

    try
    {
        if(m_Camera->grabImage(img))
            return true;
    }
    catch (exception& ex)
    {
        LOG_ERROR << "Exception grabImage..." << ex.what();
    }
    catch(...)
    {
        LOG_ERROR << "Exception";
    }
    
    return false;
}

bool Device::runSingleCapture(shared_ptr<Frame> img) {
    LOG_DEBUG << "Device::runSingleCapture";

    if(m_Camera->grabImage(img))
        return true;

    return false;

}

bool Device::setCameraSize()
{
    LOG_DEBUG << "Device::setCameraSize; "<< CameraSetting.SizeWidth <<"x"<< CameraSetting.SizeHeight ;
    
    if(!m_Camera->setSize(CameraSetting.StartX, CameraSetting.StartY, CameraSetting.SizeWidth, CameraSetting.SizeHeight, CameraSetting.CustomSize))
    {
        LOG_ERROR << "Device::setCameraSize;" <<"Fail to set camera size.";
        return false;
    }

    return true;
}

bool Device::setCameraSize(int x, int y, int w, int h) {
    LOG_DEBUG << "Device::setCameraSize;"<<w<<"x"<<h;

    if(!m_Camera->setSize(x, y, w, h, true)) {
       LOG_ERROR << "Device::setCameraSize;" << "Fail to set camera size.";
        return false;
    }

    return true;
}

/*
 * Called every frame
 */
bool Device::getCameraStatus() {
    //cout << "Device::getCameraStatus" << endl;

    return m_Camera->getStopStatus();
}

bool Device::getCameraDataSetStatus() {
    LOG_DEBUG << "Device::getCameraDataSetStatus";

    return m_Camera->getDataSetStatus();
}

bool Device::loadNextCameraDataSet(string &location) {
    LOG_DEBUG << "Device::loadNextCameraDataSet";

    return m_Camera->loadNextDataSet(location);
}

bool Device::getExposureStatus() {
    LOG_DEBUG << "Device::getExposureStatus";

    return m_Camera->isExposureAvailable();
}

bool Device::getGainStatus() {
    LOG_DEBUG << "Device::getGainStatus";

    return m_Camera->isGainAvailable();
}

Camera* Device::getCamera()
{
    return m_Camera;
}

void Device::setCamera(Camera* camera)
{
    m_Camera = camera;
}

bool Device::isStreaming()
{
    return m_Camera->isStreaming();

} 

double Device::getMinExposureTime()
{
    return m_Camera->getMinExposureTime();
}

double Device::getTemperature()
{
    return m_Camera->getTemperature();
}

bool Device::isConnected()
{
    return m_Camera->isConnected();
}

bool Device::connect()
{
    return m_Camera->connect();
}
