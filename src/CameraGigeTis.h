#pragma once
/*                          CameraGigeTis.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                       2018 Chiara Marmo
*                                   GEOPS-UPSUD
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
* \file    CameraGigeTis.h
* \author  Yoan Audureau -- Chiara Marmo -- GEOPS-UPSUD
* \version 1.2
* \date    19/03/2018
* \brief   Use Imaging source sdk to pilot GigE Cameras.
*/
//git clone https://github.com/TheImagingSource/tiscamera.git
#ifdef TISCAMERA
#include "Commons.h"

#include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <iostream>
    #include <string>
    #include "Frame.h"
    #include "TimeDate.h"
    #include "Camera.h"
    #include "EParser.h"
    #include "ECamPixFmt.h"
    #include <boost/log/common.hpp>
    #include <boost/log/expressions.hpp>
    #include <boost/log/utility/setup/file.hpp>
    #include <boost/log/utility/setup/console.hpp>
    #include <boost/log/utility/setup/common_attributes.hpp>
    #include <boost/log/attributes/named_scope.hpp>
    #include <boost/log/attributes.hpp>
    #include <boost/log/sinks.hpp>
    #include <boost/log/sources/logger.hpp>
    #include <boost/log/core.hpp>
    #include "ELogSeverityLevel.h"
    #include "tisudshl.h"
    #include <algorithm>

    #define NUMBER_OF_BUFFERS 1

    namespace freeture
    {
        class CameraDescription;
        class cameraParam;

        class CameraGigeTis : public Camera {

        private:

            DShowLib::Grabber::tVidCapDevListPtr pVidCapDevList;
            DShowLib::tIVCDRangePropertyPtr getPropertyRangeInterface(_DSHOWLIB_NAMESPACE::tIVCDPropertyItemsPtr& pItems, const GUID& id);
            bool propertyIsAvailable(const GUID& id, _DSHOWLIB_NAMESPACE::tIVCDPropertyItemsPtr m_pItemContainer);
            long getPropertyValue(const GUID& id, _DSHOWLIB_NAMESPACE::tIVCDPropertyItemsPtr m_pItemContainer);
            void setPropertyValue(const GUID& id, long val, _DSHOWLIB_NAMESPACE::tIVCDPropertyItemsPtr m_pItemContainer);
            long getPropertyRangeMin(const GUID& id, _DSHOWLIB_NAMESPACE::tIVCDPropertyItemsPtr m_pItemContainer);
            long getPropertyRangeMax(const GUID& id, _DSHOWLIB_NAMESPACE::tIVCDPropertyItemsPtr m_pItemContainer);

            DShowLib::Grabber* m_pGrabber;
            DShowLib::tFrameHandlerSinkPtr pSink;
            DShowLib::Grabber::tMemBufferCollectionPtr pCollection;
            BYTE* pBuf[NUMBER_OF_BUFFERS];

            int mFrameCounter;
            int mGain;
            double mExposure;
            double mFPS;
            CamPixFmt mImgDepth;
            int mSaturateVal;
            int mGainMin;
            int mGainMax;
            int mExposureMin;
            int mExposureMax;

        public:

            CameraGigeTis(CameraDescription, cameraParam settings);

            ~CameraGigeTis();

            bool grabSingleImage(Frame& frame);

            bool createDevice(int id);

            bool setPixelFormat(CamPixFmt format);

            void getExposureBounds(double& eMin, double& eMax);

            void getGainBounds(double& gMin, double& gMax);

            bool getFPS(double& value);

            bool setExposureTime(double value);

            bool setGain(int value);

            bool setFPS(double value);

            bool setFpsToLowerValue();

            bool acqStart();

            bool grabImage(Frame& newFrame);

            void acqStop();

            void grabCleanse();

            bool getPixelFormat(CamPixFmt& format);

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

        };
    }

#endif
