/*                      CameraGigeAravis.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2016 Yoan Audureau
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
* \file    CameraGigeAravis.h
* \author  Yoan Audureau -- Chiara Marmo GEOPS-UPSUD
* \version 1.2
* \date    19/03/2018
* \brief   Use Aravis library to pilot GigE Cameras.
*          https://wiki.gnome.org/action/show/Projects/Aravis?action=show&redirect=Aravis
*/

#pragma once
#include <iostream>
#include <string>
#include <algorithm>
#include <time.h>
#include <sstream>
#include <vector>
#include <chrono>
#include <thread>

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

#define BOOST_LOG_DYN_LINK 1

#include "config.h"
#include "ELogSeverityLevel.h"
#include "EParser.h"
#include "Camera.h"
#include "Frame.h"
#include "TimeDate.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef LINUX

#include <arv.h>
#include "arvinterface.h"
#include "CameraFirstInit.h"

    class CameraGigeAravis: public Camera
    {

        private:

            static boost::log::sources::severity_logger< LogSeverityLevel > logger;

            static class Init{

                public:

                    Init(){

                        logger.add_attribute("ClassName", boost::log::attributes::constant<std::string>("CameraGigeAravis"));

                    }

            }initializer;

            GError*         error = nullptr;        // ARAVIS API Error
            ArvCamera*      camera;                // Camera to control.
            ArvPixelFormat  pixFormat;              // Image format.
            ArvStream*      stream;                // Object for video stream reception.
            int             mStartX;                // Crop starting X.
            int             mStartY;                // Crop starting Y.
            int             mWidth;                 // Camera region's width.
            int             mHeight;                // Camera region's height.
            double          fps;                    // Camera acquisition frequency.
            double          gainMin;                // Camera minimum gain.
            double          gainMax;                // Camera maximum gain.
            unsigned int    payload;                // Width x height.
            double          exposureMin;            // Camera's minimum exposure time.
            double          exposureMax;            // Camera's maximum exposure time.
            const char*     capsString;
            int             gain;                   // Camera's gain.
            double          exp;                    // Camera's exposure time.
            bool            shiftBitsImage;         // For example : bits are shifted for dmk's frames.
            guint64         nbCompletedBuffers;     // Number of frames successfully received.
            guint64         nbFailures;             // Number of frames failed to be received.
            guint64         nbUnderruns;
            int             frameCounter;           // Counter of success received frames.

        public :

            CameraGigeAravis(bool shift);

            CameraGigeAravis();

            ~CameraGigeAravis();

            bool createDevice(int id);

            bool grabInitialization();

            void grabCleanse();

            bool acqStart();

            void acqStop();

            bool grabImage(Frame& newFrame);

            bool grabSingleImage(Frame &frame, int camID);

            bool getDeviceNameById(int id, std::string &device);

            void getExposureBounds(double &eMin, double &eMax);

            void getGainBounds(double &gMin, double &gMax);

            bool getPixelFormat(CamPixFmt &format);

            bool getFrameSize(int &x, int &y, int &w, int &h);

            bool getFPS(double &value);

            std::string getModelName();

            double getExposureTime();

            bool setExposureTime(double exp);

            bool setGain(double gain);

            bool setFPS(double fps);

            bool setFrameSize(int startx, int starty, int width, int height, bool customSize);

            bool setPixelFormat(CamPixFmt depth);

            void saveGenicamXml(std::string p);

            bool setSize(int startx, int starty, int width, int height, bool customSize);

            void getAvailablePixelFormats();

            bool FirstInitializeCamera(std::string);
    };

#endif
