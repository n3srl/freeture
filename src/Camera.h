#pragma once
/*
                                Camera.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                       2018 Chiara Marmo
*                                    GEOPS-UPSUD
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
* \file    Camera.h
* \author  Yoan Audureau -- Chiara Marmo GEOPS-UPSUD
* \version 1.2
* \date    19/03/2018
* \brief
*/

//headers refactoring ok
#include "Commons.h"

#include <string>

#include "ICameraFab.h"
#include "ECamPixFmt.h"
#include "EInputDeviceType.h"
#include "ECamSdkType.h"

#include "CameraScanner.h"
#include "Frame.h"
#include "Logger.h"

namespace freeture
{
    class Camera: public ICameraFab {

    protected:
        CameraDescription m_CameraDescriptor;
        cameraParam m_CameraSettings;

        /**
         * FEATURES
         */
        bool                m_ExposureAvailable;    // must be set to true if device can set exposure values
        bool                m_GainAvailable;        // must be set to true if device can set gain values
        
       /**
         * RUNTIME
         */
        int                 m_StartX = 0;                           // Crop starting X.
        int                 m_StartY = 0;                           // Crop starting Y.
        int                 m_Width = 0;                            // Camera region's width.
        int                 m_Height = 0;                           // Camera region's height.
        double              m_FPS = 0;                              // Camera acquisition frequency.
        double              m_MinGain = 0;                          // Camera minimum gain.
        double              m_MaxGain = 0;                          // Camera maximum gain.
        double              m_MinExposure = 0;                      // Camera's minimum exposure time.
        double              m_MaxExposure = 0;                      // Camera's maximum exposure time.
        double              m_MinFPS = 0;                           // Camera's minimum frame rate.
        double              m_MaxFPS = 0;                           // Camera's maximum frame rate.
        int                 m_Gain = 0;                             // Camera's gain.
        double              m_ExposureTime = 0;                     // Camera's exposure time.
        double              m_LastTemperature = 0.0;
        CamPixFmt           m_PixelFormat = CamPixFmt::UNDEFINED;   // Image format.
        bool                m_Streaming = false;                    //true if camera is streaming
        bool                m_ShiftBitsImage;                       // For example : bits are shifted for dmk's frames.

        /**
         * METRICS
         */
        int64_t             m_MissingPacketCount;   //number of missing packets
        
    public:

        Camera(CameraDescription camera_descriptor, cameraParam settings):
            m_CameraDescriptor (camera_descriptor),
            m_CameraSettings(settings),
            m_StartX(settings.ACQ_STARTX),
            m_StartY(settings.ACQ_STARTY),
            m_Width(settings.ACQ_WIDTH),
            m_Height(settings.ACQ_HEIGHT),
            m_FPS(settings.ACQ_FPS),
            m_ShiftBitsImage(settings.SHIFT_BITS),
            m_Gain(settings.ACQ_DAY_GAIN),
            m_ExposureTime(settings.ACQ_DAY_EXPOSURE)
        {}

        virtual ~Camera() {};

        virtual void getAvailablePixelFormats() {};

        /**
        * Get informations about a specific device.
        *
        */
        virtual bool getInfos() { return false; };

        virtual bool getCameraName() { return false; };

        InputDeviceType getDeviceType() { return m_CameraDescriptor.DeviceType; };

        /**
        * Get device's grabbing status.
        *
        * @return Device grabs frames or not.
        */
        virtual bool getStopStatus() { return false; };

        /**
        * Prepare device to grab frames.
        *
        * @return Success status to prepare camera.
        */
        virtual bool grabInitialization() { return false; };

        /**
        * Run acquisition in continuous mode.
        *
        */
        virtual bool acqStart() { return false; };

        /**
        * Run acquisition choose if continuous or single
        */
        virtual bool acqStart(bool) { return false; };

        /**
        * Stop acquisition.
        *
        */
        virtual void acqStop() {};

        /**
        * Close a device and clean resources.
        *
        */
        virtual void grabCleanse() { destroyDevice(); };

        /**
        * Get a frame from continuous acquisition.
        *
        * @param newFrame New frame's container object.
        * @return Success status to grab a frame.
        */
        virtual bool grabImage(Frame& newFrame) { return false; };

        /**
        * Get a frame from single acquisition.
        *
        * @param newFrame Frame's container object.
        * @return Success status to grab a frame.
        */
        virtual bool grabSingleImage(Frame& frame) { return false; };

        /**
        * Get device's exposure time bounds.
        *
        * @param eMin Return minimum exposure time value.
        * @param eMax Return maximum exposure time value.
        */
        virtual void getExposureBounds(double& eMin, double& eMax) {};
        /**
        * Get device's frame acquisition bounds.
        *
        * @param eMin Return minimum frame acquisition value.
        * @param eMax Return maximum frame acquisition value.
        */
        virtual void getFPSBounds(double& fMin, double& fMax) {};


        /**
        * Get device's gain bounds.
        *
        * @param gMin Return minimum gain value.
        * @param gMax Return maximum gain value.
        */
        virtual void getGainBounds(double& gMin, double& gMax) {};

        /**
        * Get device's image format.
        *
        * @param format Return image format.
        * @return Success status to get format.
        */
        virtual bool getPixelFormat(CamPixFmt& format) { return false; };

        /**
        * Get device's frame size.
        *
        * @param frame's width
        * @param frame's height
        * @return Success to get frame'size.
        */
        virtual bool getFrameSize(int& x, int& y, int& w, int& h) { return false; };

        /**
        * Get device's acquisition frequency.
        *
        * @return Device's fps.
        */
        virtual bool getFPS(double& value) { return false; };

        /**
        * Get FPS enumeration values.
        *
        * @return Possible fps values.
        */
        virtual bool getFpsEnum(std::vector<double>& values) { return false; };

        /**
        * Get device's model name.
        *
        * @return Device's model name.
        */
        virtual std::string getModelName() { return ""; };

        /**
        * Get device's gain value.
        *
        * @return Device's gain.
        */
        virtual double getGain() { return 0; };

        /**
        * Get device's exposure time value.
        *
        * @return Device's exposure time.
        */
        virtual double getExposureTime() { return 0.0; };

        /**
        * Set device's exposure time value.
        *
        * @param value New exposure time value (us).
        * @return Success status to set new exposure time.
        */
        virtual bool setExposureTime(double value) { return false; };

        /**
        * Set device's gain value.
        *
        * @param value New gain value.
        * @return Success status to set new gain.
        */
        virtual bool setGain(double value) { return false; };

        virtual bool setAutoExposure(bool val) { return false; };

        /**
        * Set device's acquisition frequency.
        *
        * @param value New fps value.
        * @return Success status to set fps.
        */
        virtual bool setFPS(double value) { return false; };

        virtual bool setSize(int startx, int starty, int width, int height, bool customSize) { return false; };

        /**
        * Set device's format.
        *
        * @param format New format.
        * @return Success status to set format.
        */
        virtual bool setPixelFormat() { return false; };

        /**
        * Get data status if a set of directories or videos are used in input.
        *
        * @return If there is still recorded frames to load in input.
        */
        virtual bool getDataSetStatus() { return false; };

        virtual double getMinExposureTime() = 0;

        /**
        * Load next data set of frames.
        *
        * @return Success status to load next data set.
        */
        virtual bool loadNextDataSet(std::string& location) { location = ""; return true; };

        virtual void test() { LOG_DEBUG << "Camera::test"; };

        virtual bool FirstInitializeCamera(std::string) {
            LOG_DEBUG << "Camera::FirstInitializeCamera; Inizialize camera done";
            return true;
        };

        virtual bool isStreaming()
        {
            LOG_DEBUG << "Camera::IsStreaming";
            return m_Streaming;
        }

        bool isExposureAvailable() {
            LOG_DEBUG << "Camera::isExposureAvailable";
            return m_ExposureAvailable;
        }

        bool isGainAvailable() {
            LOG_DEBUG << "Camera::isGainAvailable";
            return m_GainAvailable;
        }

    };
}