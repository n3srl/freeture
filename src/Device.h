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

#include "CameraSettings.h"
#include "EAcquisitionMode.h"


namespace freeture
{
    class Camera;
    class Frame;

    class Device {
    private:
        framesParam m_FramesParam;
        videoParam  m_VideoParam;
        Camera* m_Camera = nullptr;


    private:
        bool setCameraExposureTime();
        bool setCameraGain();
        bool setCameraFPS();
        bool setCameraSize();
        bool setCameraPixelFormat();
        Camera* getCamera();

    public:
        bool mVideoFramesInput; // TRUE if input is a video file or frames directories.

        /// <summary>
        /// This is the current camera settings, used to run single or continuous shot.
        /// Need to be configured according to sun time from acquisition thread.
        /// </summary>
        CameraSettings CameraSetting;

        Device();
        ~Device();

        void Setup(cameraParam cp, framesParam fp, videoParam vp);

        bool runContinuousCapture(Frame& img);
        bool runSingleCapture(Frame& img);

        /// <summary>
        /// Call camera acqStart. 
        /// Start camera streaming
        /// </summary>
        /// <returns></returns>
        bool startCamera(EAcquisitionMode);

        /// <summary>
        /// Call camera acqStop
        /// Stop camera streaming.
        /// </summary>
        /// <returns></returns>
        bool stopCamera();

        bool getDeviceName();
        InputDeviceType getDeviceType();
 
        bool setCameraExposureTime(double value);
        bool setCameraGain(double value);
        bool setCameraFPS(double value);
        bool setCameraSize(int x, int y, int w, int h);

        bool getCameraStatus();
        bool getCameraDataSetStatus();
        bool getSupportedPixelFormats();
        bool loadNextCameraDataSet(std::string& location);
        bool getExposureStatus();
        bool getGainStatus();

        bool firstIinitializeCamera(std::string);

        /// <summary>
        /// Call grabInitialization on camera
        ///     - set fps
        ///     - set gain
        ///     - set exposure time
        ///     - initialize stream parameters
        /// </summary>
        /// <returns></returns>
        bool initializeCamera();

        /// <summary>
        /// Used by factory to set camera instance to this device control
        /// </summary>
        /// <param name=""></param>
        void setCamera(Camera*);

        bool isStreaming();
        
        double getMinExposureTime();
    };
}
