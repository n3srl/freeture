#pragma once
/*
                                CameraWindows.h

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
*   Last modified:      02/10/2015
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    CameraWindows.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    02/10/2015
*/
#include "Commons.h"


#ifdef VIDEOINPUT

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "videoInput.h"
#include "Frame.h"
#include "Camera.h"

namespace freeture
{
    class CameraDescription;

    class CameraWindows : public Camera {

    private:

        int mDevNumber;
        videoInput mVideoInput;
        int mStartX;
        int mStartY;
        int mWidth;
        int mHeight;
        int mSize;
        unsigned char* mBuffer;
        int mFrameCounter;

        // see : http://msdn.microsoft.com/en-us/library/dd318253(v=vs.85).aspx
        // and : http://msdn.microsoft.com/en-us/library/dd389148(v=vs.85).aspx
        typedef enum {

            CameraControl_Pan,
            CameraControl_Tilt,
            CameraControl_Roll,
            CameraControl_Zoom,
            CameraControl_Exposure,
            CameraControl_Iris,
            CameraControl_Focus

        }CameraControlProperty;

        // see : http://msdn.microsoft.com/en-us/library/dd318251(v=vs.85).aspx
        typedef enum {

            CameraControl_Flags_Auto = 0x0001,
            CameraControl_Flags_Manual = 0x0002

        }CameraControlFlags;

        static const long mDefaultFocus = 0;

    public:

        CameraWindows(CameraDescription, cameraParam);

        ~CameraWindows();

        vector<pair<int, string>> getCamerasList();

        bool grabSingleImage(Frame& frame);

        bool createDevice(int id);

        bool setPixelFormat(CamPixFmt format);

        void getExposureBounds(double& eMin, double& eMax);

        void getGainBounds(int& gMin, int& gMax);

        bool getFPS(double& value);

        bool setExposureTime(double value);

        bool setGain(int value);

        bool setFPS(double value);

        bool setSize(int startx, int starty, int width, int height, bool customSize);

        bool setFpsToLowerValue();

        bool acqStart();

        bool grabImage(Frame& newFrame);

        void acqStop();

        void grabCleanse();

        bool getPixelFormat(CamPixFmt& format);

        double getExposureTime();

        //ABSTRACT FACTORY METHODS
        /// <summary>
        /// initialize SDK
        /// </summary>
        /// <returns></returns>
        bool initSDK() override;

        /// <summary>
        /// init once, run configuration once (use configuration file)
        /// </summary>
        /// <returns></returns>
        bool initOnce() override;

        /// <summary>
        /// init the camera, eg. running functions when created 
        /// CALL GRAB INITIALIZATION 
        /// </summary>
        /// <returns></returns>
        bool init() override;

        /// <summary>
        /// DEPRECATED USE INIT INSTEAD.
        /// 
        /// </summary>
        /// <returns></returns>
        bool grabInitialization() override;

        /// <summary>
        /// retreive main camera boundaries upon configuration: 
        ///     - fps
        ///     - gain
        ///     - exposure time
        /// </summary>
        void fetchBounds(parameters&) override;

        /// <summary>
        /// configure the camera with the given parameters
        /// </summary>
        /// <param name=""></param>
        void configure(parameters&) override;

        /// <summary>
        /// check if configuration is allowed
        /// </summary>
        /// <param name=""></param>
        /// <returns></returns>
        bool configurationCheck(parameters&) override;

        double getMinExposureTime() override;

    };
}
#endif
