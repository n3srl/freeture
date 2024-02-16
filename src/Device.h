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

    class Device {

    public:

        bool mVideoFramesInput; // TRUE if input is a video file or frames directories.

    private:
//        std::vector<std::pair<int, std::pair<int, CamSdkType>>> mDevices;

        bool        mCustomSize;
        int         mStartX;
        int         mStartY;
        int         mSizeWidth;
        int         mSizeHeight;
        int         mNightExposure;
        int         mNightGain;
        int         mDayExposure;
        int         mDayGain;
        int         mFPS; // NEED TO BE DOUBLE
        bool        mVerbose;

        framesParam m_FramesParam;
        videoParam  m_VideoParam;

    public:
        double      minExposureTime = 0.0;
        double      maxExposureTime = 0.0;
        double      minFPS = 0.0;
        double      maxFPS = 0.0;
        double      minGain = 0.0;
        double      maxGain = 0.0;


    public:
        //int         mCamID;         // ID in a specific sdk.
        Camera*         mCam = nullptr;
        CamPixFmt       mFormat;
        InputDeviceType mDeviceType;
        std::string     mCfgPath;

        void Setup(cameraParam cp, framesParam fp, videoParam vp);

        Device();

        ~Device();

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
        bool initializeCamera();

    private:
      

    };
}
