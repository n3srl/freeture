#pragma once
/*
                                Device.h

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
* \file    Device.h
* \author  Yoan Audureau -- Chiara Marmo -- GEOPS-UPSUD
* \version 1.2
* \date    19/03/2018
* \brief
*/
//header refactoring ok
#include "Commons.h"

#include <string>
#include <vector>

#include "CameraScanner.h"
#include "EInputDeviceType.h"
#include "ECamPixFmt.h"
#include "ECamSdkType.h"

#include "SParam.h"

// #include "ELogSeverityLevel.h"
// #include "EImgBitDepth.h"
// #include "ECamPixFmt.h"
// #include "EParser.h"
// #include "Conversion.h"
// #include "Camera.h"
// #include "CameraGigeAravis.h"
// #include "CameraGigePylon.h"
// #include "CameraGigeTis.h"
// #include "CameraVideo.h"
// #include "CameraV4l2.h"
// #include "CameraFrames.h"
// #include "CameraWindows.h"
// #include "EInputDeviceType.h"
// #include "ECamSdkType.h"
// #include "SParam.h"

namespace freeture
{
    class Camera;
    class Frame;

#define ARENA_SDK false    //Set this to true to use Arena SDK instead of aravis for LUCID cameras

    class Device {

    public:

        bool mVideoFramesInput; // TRUE if input is a video file or frames directories.

    private:
        std::vector<CameraDescription> listCams;

        std::vector<std::pair<int, std::pair<int, CamSdkType>>> mDevices;

        bool        mCustomSize;
        int         mStartX;
        int         mStartY;
        int         mSizeWidth;
        int         mSizeHeight;
        int         mNightExposure;
        int         mNightGain;
        int         mDayExposure;
        int         mDayGain;
        int         mFPS;



        bool        mShiftBits;
        bool        mVerbose;
        framesParam mfp;
        videoParam  mvp;

        void mergeList(std::vector<CameraDescription>&);
    public:
        int         mCamID;         // ID in a specific sdk.
        int         mGenCamID;      // General ID.
        Camera* mCam;
        int         mNbDev;
        CamPixFmt   mFormat;
        std::string      mCfgPath;
        InputDeviceType mDeviceType;
        double      mMinExposureTime;
        double      mMaxExposureTime;
        double      mMinFPS;
        double      mMaxFPS;
        double         mMinGain;
        double         mMaxGain;
        //int         mNbFrame;

        void Setup(cameraParam cp, framesParam fp, videoParam vp, int cid);

        Device();

        ~Device();

        CamSdkType getDeviceSdk(int id);

        void listDevices(bool printInfos);

        std::vector<CameraDescription> getListDevice();

        bool createCamera(int id, bool create);

        bool createCamera();

        bool initializeCamera();

        bool runContinuousCapture(Frame& img);

        bool runSingleCapture(Frame& img);

        bool startCamera();

        bool stopCamera();

        bool setCameraPixelFormat();

        bool getCameraGainBounds(double& min, double& max);

        void getCameraGainBounds();

        bool getCameraExposureBounds(double& min, double& max);

        void getCameraExposureBounds();

        bool getCameraFPSBounds(double& min, double& max);

        void getCameraFPSBounds();

        bool getDeviceName();

        bool recreateCamera();

        InputDeviceType getDeviceType();

        bool setCameraAutoExposure(bool);

        bool setCameraNightExposureTime();

        bool setCameraDayExposureTime();

        bool setCameraNightGain();

        bool setCameraDayGain();

        bool setCameraExposureTime(double value);

        bool setCameraGain(double value);

        bool setCameraFPS(double value);

        bool setCameraFPS();

        bool setCameraSize();

        bool getCameraFPS(double& fps);

        bool getCameraStatus();

        bool getCameraDataSetStatus();

        bool getSupportedPixelFormats();

        bool loadNextCameraDataSet(std::string& location);

        bool getExposureStatus();

        bool getGainStatus();

        bool setCameraSize(int x, int y, int w, int h);
        bool getDeviceCameraSizeParams(int& x, int& y, int& height, int& width);

        int getNightExposureTime() { return mNightExposure; };
        int getNightGain() { return mNightGain; };
        int getDayExposureTime() { return mDayExposure; };
        int getDayGain() { return mDayGain; };

        void setVerbose(bool status);

        Camera* getCamera();
        bool firstIinitializeCamera(std::string);

    private:

        bool createDevicesWith(CamSdkType sdk);

    };
}
