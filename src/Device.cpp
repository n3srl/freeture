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

void Device::Setup(cameraParam cp, framesParam fp, videoParam vp, int cid)
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
    mMinExposureTime = -1;
    mMaxExposureTime = -1;
    mMinGain = -1;
    mMaxGain = -1;
    mShiftBits = cp.SHIFT_BITS;
    mFormat = cp.ACQ_FORMAT;
    mCustomSize = cp.ACQ_RES_CUSTOM_SIZE;
    mStartX = cp.ACQ_STARTX;
    mStartY = cp.ACQ_STARTY;
    mSizeWidth = cp.ACQ_WIDTH;
    mSizeHeight = cp.ACQ_HEIGHT;
    mDeviceType = UNDEFINED_INPUT_TYPE;
    mvp = vp;
    mfp = fp;
    //mNbFrame = 0;

}


Device::Device() {
    LOG_DEBUG << "Device::Device";

    mFormat         = MONO8;
    mNightExposure  = 0;
    mNightGain      = 0;
    mDayExposure    = 0;
    mDayGain        = 0;
    mMinExposureTime = -1;
    mMaxExposureTime = -1;
    mMinGain = -1;
    mMaxGain = -1;
    mFPS            = 30;
    mCamID          = 0;
    mGenCamID       = 0;
    mCustomSize     = false;
    mStartX         = 0;
    mStartY         = 0;
    mSizeWidth      = 1440;
    mSizeHeight     = 1080;
    mCam            = nullptr;
    mVideoFramesInput = false;
    mShiftBits      = false;
    mNbDev          = -1;
    mVerbose        = true;
    mDeviceType     = UNDEFINED_INPUT_TYPE;
    vector<string> finput, vinput;
    mvp.INPUT_VIDEO_PATH = vinput;
    mfp.INPUT_FRAMES_DIRECTORY_PATH = finput;
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
    Device* device = m_CameraDeviceManager->getDevice();
    if(device != NULL)
    {
        return device->mCam->FirstInitializeCamera(m_ConfigurationFilePath);
    }
    return false;
}

bool Device::createCamera(int id, bool create)
{
    LOG_DEBUG << "Device::createCamera";
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    
    Device* device = m_CameraDeviceManager->getDevice();
    
    if(id >=0 && id <= m_CameraDeviceManager->deviceCount - 1)
    {
        CameraDescription camera = m_CameraDeviceManager->getListDevice().at(id);
        LOG_DEBUG << "CREATE CAMERA " + camera.Description;
        // Create Camera object with the correct sdk.
        if ( !createDevicesWith(camera.Sdk) )
        {
            LOG_DEBUG << "Fail to select correct sdk"<< endl;
            EParser<CamSdkType> parser;
            LOG_DEBUG << "Fail to select correct sdk : "<<parser.getStringEnum( mDevices.at(id).second.second )<< endl;
            return false;
        }
    

        device->mCamID = camera.Id;
        if(device->mCam != nullptr)
        {
                
                if(!device->mCam->createDevice(device->mCamID))
                {
                    LOG_ERROR << "Fail to create device with ID  : " << id;
                    device->mCam->grabCleanse();
                    return false;
                }
                LOG_DEBUG <<  "Device::createCamera(int,bool) - created new camera";
            return true;
        } else {
            LOG_DEBUG << "Device::createCamera(int,bool) - camera already exists";
            return true;
        }
        
    }

    LOG_ERROR << "No device with ID " << id;

    return false;

}

bool Device::createCamera()
{
    LOG_DEBUG << "Device::createCamera";

    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    Device* device = m_CameraDeviceManager->getDevice();

    if(mGenCamID >=0 && mGenCamID < m_CameraDeviceManager->deviceCount) {

        // Create Camera object with the correct sdk.
        
        try {
            
            if(!createDevicesWith(m_CameraDeviceManager->getListDevice().at(mGenCamID).Sdk)) {
                LOG_ERROR <<  "CREATE CAMERA ERROR " << endl;
            return false;
        } 
        } catch (exception &e) {
            LOG_DEBUG << "I FOUND EXC" << endl;
            LOG_DEBUG << e.what() << endl;
            return false;
        }   
            
        mCamID = m_CameraDeviceManager->getListDevice().at(mGenCamID).Id;
        
        if(device->mCam != nullptr)
        {
            
            if(!device->mCam->createDevice(mCamID))
            {
                LOG_ERROR << "Device::createCamera;" << "Fail to create device with ID  : " << mGenCamID;
                device->mCam->grabCleanse();
                return false;
            }
            LOG_DEBUG << "Device::createCamera; Created new camera";
            return true;
        }
        else
        {
            LOG_DEBUG << "Device::createCamera; Camera already exists";
            return true;
        }
    }

    LOG_DEBUG << "Device::createCamera;" << "No s with ID " << mGenCamID << "DEVICES " << m_CameraDeviceManager->deviceCount << endl;
    LOG_ERROR << "Device::createCamera;" << "No device with ID " << mGenCamID;

    return false;

}

bool Device::recreateCamera() {
    LOG_DEBUG << "Device::recreateCamera";
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    if (mGenCamID >= 0 && mGenCamID < m_CameraDeviceManager->deviceCount) {

        mCamID = m_CameraDeviceManager->getListDevice().at(mGenCamID).Id;

        if(mCam != nullptr) {
            if(!mCam->recreateDevice(mCamID)){
                LOG_ERROR << "Fail to create device with ID  : " << mGenCamID;
                mCam->grabCleanse();
                return false;
            }
            LOG_DEBUG << "Device::recreateCamera - created new camera";
            return true;
        } else {
            LOG_DEBUG << "Device::recreateCamera - camera already exists";
            return true;
         }

    }

    LOG_ERROR << "No device with ID " << mGenCamID;

    return false;

}

void Device::setVerbose(bool status) {
    LOG_DEBUG << "Device::setVerbose";
    mVerbose = status;

}


CamSdkType Device::getDeviceSdk(int id) {
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    vector<CameraDescription> devices = m_CameraDeviceManager->getListDevice();
    
    if(id >=0 && id <= (m_CameraDeviceManager->deviceCount - 1)) {
        return devices.at(id).Sdk;
    }
    LOG_DEBUG << "ERROR GETTING DEVICE SDK FOR CAMERA ID " << id << " NUM OF DEVICES " << ( m_CameraDeviceManager->deviceCount - 1) << endl;
}

bool Device::createDevicesWith(CamSdkType sdk) {
    LOG_DEBUG << "Device::createDevicesWith" ;

    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    Device* device = m_CameraDeviceManager->getDevice();
    
    if (device->mCam!=nullptr)
    {
        
        LOG_ERROR << "Device::createDevicesWith;"<< "MEMORY LEAKING";
        assert (device->mCam != nullptr);
        free(device->mCam);

        return false;
    }


    switch (sdk)
    {
    case CamSdkType::VIDEOFILE:
    {
        mVideoFramesInput = true;
        device->mCam = new CameraVideo(mvp.INPUT_VIDEO_PATH, mVerbose);
        device->mCam->grabInitialization();
    }

    break;

    case CamSdkType::FRAMESDIR:

    {
        mVideoFramesInput = true;
        // Create camera using pre-recorded fits2D in input.
        device->mCam = new CameraFrames(mfp.INPUT_FRAMES_DIRECTORY_PATH, 1, mVerbose);
        if (!device->mCam->grabInitialization())
            throw "Fail to prepare acquisition on the first frames directory.";

    }

    break;

    case CamSdkType::V4L2:

    {
        LOG_DEBUG << " Device::createDevicesWith;" << "SDK: V4L2";
#ifdef LINUX
        device->mCam = new CameraV4l2();
#endif
    }

    break;

    case CamSdkType::VIDEOINPUT:

    {
        LOG_DEBUG << " Device::createDevicesWith;" << "SDK: VIDEOINPUT";
#ifdef VIDEOINPUT
        device->mCam = new CameraWindows();
#endif
    }

    break;

    case CamSdkType::ARAVIS:

    {
        LOG_DEBUG << " Device::createDevicesWith;" << "SDK: ARAVIS";
#ifdef LINUX
        device->mCam = new CameraGigeAravis(mShiftBits);
#endif
    }

    break;

    case CamSdkType::LUCID_ARAVIS:

    {
        LOG_DEBUG << " Device::createDevicesWith;" << "SDK: LUCID_ARAVIS";
#ifdef LINUX

        device->mCam = new CameraLucidArena(mShiftBits);


#endif
    }

    break;

    case CamSdkType::LUCID_ARENA:

    {
        LOG_DEBUG << " Device::createDevicesWith;" << "SDK: LUCID_ARENA";
        device->mCam = new CameraLucidArena_PHX016S(mShiftBits);

    }

    break;

    case CamSdkType::PYLONGIGE:

    {
        LOG_DEBUG << "SDK: PYLONGIGE";
#ifdef USE_PYLON
        device = new CameraGigePylon();
#endif
    }

    break;

    case CamSdkType::TIS:

    {
        LOG_DEBUG << "SDK: TIS";
#ifdef TISCAMERA
        device = new CameraGigeTis();
#endif
    }

    break;

    default:

        LOG_ERROR << "Device::createDevicesWith; Unknown SDK:";
        return false;
    }

    if (device->mCam == nullptr) {
        LOG_ERROR << "Device::createDevicesWith;" << "Failed to create camera";
        return false;
    }

    return true;

}

void Device::mergeList( vector<CameraDescription>& Devices )
{
        //foreach device found test if already exists. if not add to list
        for (int i=0;i<Devices.size();i++)
        {
            bool add_to_list = true;

            for (int j=0;j<listCams.size();j++)
                if (listCams[j].Address==Devices[i].Address && listCams[j].Sdk == Devices[i].Sdk) 
                {
                    add_to_list = false;
                    break;
                }

            if (add_to_list)
                
                listCams.push_back(Devices[i]);
        }
}

void Device::listDevices(bool printInfos)
{
    LOG_DEBUG << "Device::listDevices";

    int nbCam = 0;
    mNbDev = 0;
    pair<int, CamSdkType> elem;             // general index to specify camera to use
    pair<int,pair<int,CamSdkType>> subElem; // index in a specific sdk

// 
//         // PYLONGIGE
// #ifdef USE_PYLON
//         mCam = new CameraGigePylon();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
//             elem.first = mNbDev; elem.second = CamSdkType::PYLONGIGE;
//             subElem.first = listCams.at(i).first; subElem.second = elem;
//             mDevices.push_back(subElem);
//             if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second << endl;
//             mNbDev++;
//         }
//         delete mCam;
// #endif
// #ifdef TISCAMERA
//         // TIS
// 
//         mCam = new CameraGigeTis();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
//             elem.first = mNbDev; elem.second = CamSdkType::TIS;
//             subElem.first = listCams.at(i).first; subElem.second = elem;
//             mDevices.push_back(subElem);
//             if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second << endl;
//             mNbDev++;
//         }
//         delete mCam;
// #endif
// #ifdef VIDEOINPUT
//         // WINDOWS
// 
//         mCam = new CameraWindows();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
// 
//             // Avoid to list basler
//             string::size_type pos1 = listCams.at(i).second.find("Basler");
//             string::size_type pos2 = listCams.at(i).second.find("BASLER");
//             if((pos1 != string::npos) || (pos2 != string::npos)) {
//                 //LOG_DEBUG << "found \"words\" at position " << pos1 << endl;
//             } else {
//                 elem.first = mNbDev; elem.second = CamSdkType::VIDEOINPUT;
//                 subElem.first = listCams.at(i).first; subElem.second = elem;
//                 mDevices.push_back(subElem);
//                 if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second << endl;
//                 mNbDev++;
//             }
//         }
// 
//         delete mCam;
// #endif
// 
        //PRECEDENTE TO LUCID_ARAVIS
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    m_CameraDeviceManager->listDevice(true);

        // V4L
        //CameraScanner* v4l_scanner = Camera::Scanner->CreateScanner(CamSdkType::V4L2);
        //assert(v4l_scanner!=nullptr);
        // v4l_scanner->getCamerasList();
        //listCams.insert ( listCams.end(), v4l_scanner->Devices.begin(),v4l_scanner->Devices.end());

    // VIDEO
    /*
    elem.first = mNbDev; elem.second = VIDEOFILE;
    subElem.first = 0; subElem.second = elem;
    mDevices.push_back(subElem);
    if(printInfos) cout << "[" << mNbDev << "]    VIDEO FILES" << endl;
    mNbDev++;

    // FRAMES

    elem.first = mNbDev; elem.second = FRAMESDIR;
    subElem.first = 0; subElem.second = elem;
    mDevices.push_back(subElem);
    if(printInfos) cout << "[" << mNbDev << "]    FRAMES DIRECTORY" << endl;
    mNbDev++;
    */
}


vector<CameraDescription> Device::getListDevice()
{
    LOG_DEBUG << "Device::getListDevice";

    int nbCam = 0;
    mNbDev = 0;
    pair<int, CamSdkType> elem;             // general index to specify camera to use
    pair<int,pair<int,CamSdkType>> subElem; // index in a specific sdk

    #ifdef WINDOWS

        // PYLONGIGE
#ifdef USE_PYLON
//         mCam = new CameraGigePylon();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
//             elem.first = mNbDev; elem.second = PYLONGIGE;
//             subElem.first = listCams.at(i).first; subElem.second = elem;
//             mDevices.push_back(subElem);
//             if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second << endl;
//             mNbDev++;
//         }
//         delete mCam;
#endif
#ifdef TISCAMERA
//         // TIS
// 
//         mCam = new CameraGigeTis();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
//             elem.first = mNbDev; elem.second = TIS;
//             subElem.first = listCams.at(i).first; subElem.second = elem;
//             mDevices.push_back(subElem);
//             if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second << endl;
//             mNbDev++;
//         }
//         delete mCam;
#endif
#ifdef VIDEOINPUT
//         // WINDOWS
// 
//         mCam = new CameraWindows();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
// 
//             // Avoid to list basler
//             string::size_type pos1 = listCams.at(i).second.find("Basler");
//             string::size_type pos2 = listCams.at(i).second.find("BASLER");
//             if((pos1 != string::npos) || (pos2 != string::npos)) {
//                 //LOG_DEBUG << "found \"words\" at position " << pos1 << endl;
//             } else {
//                 elem.first = mNbDev; elem.second = VIDEOINPUT;
//                 subElem.first = listCams.at(i).first; subElem.second = elem;
//                 mDevices.push_back(subElem);
//                 if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second << endl;
//                 mNbDev++;
//             }
//         }
// 
//         delete mCam;
#endif
#endif
   
#ifdef USE_ARAVIS
        CameraScanner* lucid_aravis_scanner = Camera::Scanner->CreateScanner(CamSdkType::LUCID_ARAVIS);
        assert(lucid_aravis_scanner!=nullptr);
        lucid_aravis_scanner->getCamerasList();
        listCams.insert ( listCams.end(), lucid_aravis_scanner->Devices.begin(),lucid_aravis_scanner->Devices.end());
#endif

#ifdef USE_ARENA
        //ARENA SDK
        CameraScanner* lucid_arena_scanner = Camera::Scanner->CreateScanner(CamSdkType::LUCID_ARENA);
        assert(lucid_arena_scanner!=nullptr);
        lucid_arena_scanner->getCamerasList();

        mergeList(lucid_arena_scanner->Devices);
#endif

#ifdef USE_ARAVIS
        // ARAVIS
        CameraScanner* aravis_scanner = Camera::Scanner->CreateScanner(CamSdkType::ARAVIS);
        assert(aravis_scanner!=nullptr);
        aravis_scanner->getCamerasList();

        mergeList(aravis_scanner->Devices);
#endif

    // VIDEO
    /*
    elem.first = mNbDev; elem.second = VIDEOFILE;
    subElem.first = 0; subElem.second = elem;
    mDevices.push_back(subElem);
    if(printInfos) cout << "[" << mNbDev << "]    VIDEO FILES" << endl;
    mNbDev++;

    // FRAMES

    elem.first = mNbDev; elem.second = FRAMESDIR;
    subElem.first = 0; subElem.second = elem;
    mDevices.push_back(subElem);
    if(printInfos) cout << "[" << mNbDev << "]    FRAMES DIRECTORY" << endl;
    mNbDev++;
    */

   return listCams;
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

    mCam->getExposureBounds(mMinExposureTime, mMaxExposureTime);

}

bool Device::getCameraFPSBounds(double &min, double &max) {
    LOG_DEBUG << "Device::getCameraFPSBounds(double,double)";
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();

    m_CameraDeviceManager->getDevice()->mCam->getFPSBounds(min, max);
    

    return true;
}

void Device::getCameraFPSBounds() {
    LOG_DEBUG << "Device::getCameraFPSBounds";
    mCam->getFPSBounds(mMinFPS, mMaxFPS);
}


bool Device::getCameraGainBounds(double &min, double &max) {
    LOG_DEBUG << "Device::getCameraGainBounds(double,double)";


    mCam->getGainBounds(min, max);
    return true;
}

void Device::getCameraGainBounds() {
    LOG_DEBUG << "Device::getCameraGainBounds";

    mCam->getGainBounds(mMinGain, mMaxGain);
}

bool Device::setCameraNightExposureTime()
{   
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();

    Device* device = m_CameraDeviceManager->getDevice();
    
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
    LOG_INFO << "single capture with cam: " << mCamID << endl;
    if(mCam->grabSingleImage(img, mCamID))
        return true;

    return false;

}

bool Device::getDeviceCameraSizeParams(int& x,int& y,int& height,int& width)
{
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    Device* dev = m_CameraDeviceManager->getDevice();
    x = dev->mStartX;
    y = dev->mStartY;
    height = dev->mSizeHeight;
    width = dev->mSizeWidth;
    return true;
}

bool Device::setCameraSize()
{
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    Device* device = m_CameraDeviceManager->getDevice();
    LOG_DEBUG << "Device::setCameraSize; "<< device->mSizeWidth <<"x"<< device->mSizeHeight ;
    
    Camera* mCam = device->mCam;
    
    if(!mCam->setSize(device->mStartX, device->mStartY, device->mSizeWidth, device->mSizeHeight, device->mCustomSize)) 
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

    return mCam->mExposureAvailable;
}

bool Device::getGainStatus() {
    LOG_DEBUG << "Device::getGainStatus";

    return mCam->mGainAvailable;
}

Camera* Device::getCamera()
{
    return mCam;
}
