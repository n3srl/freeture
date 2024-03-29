#pragma once
/*
                            CameraGigePylon.h

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
 * \file    CameraGigePylon.cpp
 * \author  Yoan Audureau -- Chiara Marmo -- GEOPS-UPSUD
 * \version 1.2
 * \date    19/03/2018
 * \brief   Use Pylon library to pilot GigE Cameras.
 */

#include "Commons.h"

#ifdef USE_PYLON


#include "config.h"


    #include "Frame.h"
    #include "TimeDate.h"
    #include "Conversion.h"
    #include "SaveImg.h"
    #include "Camera.h"
    #include "ECamPixFmt.h"
    #include "EParser.h"
    #include <boost/log/common.hpp>
    #include <boost/log/attributes.hpp>
    #include <boost/log/sources/logger.hpp>
    #include <boost/log/core.hpp>
    #include "ELogSeverityLevel.h"

    #include <pylon/PylonIncludes.h>
    #include <pylon/gige/BaslerGigEInstantCamera.h>
    #include <pylon/gige/BaslerGigECamera.h>

    using namespace Pylon;
    using namespace GenApi;
    using namespace Basler_GigECameraParams;

    static const uint32_t nbBuffers = 20; // Buffer's number used for grabbing
    namespace freeture
    {
        class CameraGigePylon : public Camera {

        private:

            // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
            // is initialized during the lifetime of this object.
            Pylon::PylonAutoInitTerm                autoInitTerm;

            uint8_t* ppBuffersUC[nbBuffers];         // Buffer for the grabbed images in 8 bits format.
            uint16_t* ppBuffersUS[nbBuffers];         // Buffer for the grabbed images in 16 bits format.
            StreamBufferHandle                      handles[nbBuffers];
            CTlFactory* pTlFactory;
            ITransportLayer* pTl;                    // Pointer on the transport layer.
            CBaslerGigECamera* pCamera;                       // Pointer on basler camera.
            CBaslerGigECamera::StreamGrabber_t* pStreamGrabber;
            DeviceInfoList_t                        devices;
            GrabResult                              result;
            bool                                    connectionStatus;
            int                                     mFrameCounter;

        public:

            CameraGigePylon(CameraDescription, cameraParam settings);

            ~CameraGigePylon(void);

            bool createDevice(int id);

            bool getDeviceNameById(int id, string& device);

            void grabCleanse();

            bool acqStart();

            void acqStop();

            bool grabImage(Frame& newFrame);

            bool grabSingleImage(Frame& frame);

            void getExposureBounds(double& eMin, double& eMax);

            void getGainBounds(double& gMin, double& gMax);

            bool getPixelFormat(CamPixFmt& format);

            bool getFrameSize(int& w, int& h);

            bool getFPS(double& value);

            string getModelName();

            bool setExposureTime(double exp);

            bool setGain(int gain);

            bool setFPS(double fps);

            bool setPixelFormat(CamPixFmt format);

            double getExposureTime();

            bool setSize(int x, int y, int width, int height, bool customSize);

            void getAvailablePixelFormats();


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
