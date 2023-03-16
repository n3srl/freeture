/**
* \file    CameraLucidArena.h
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief   Use Aravis library to pilot Lucid phoenix cameras.
*
*/

#pragma once

#include "config.h"

#ifdef LINUX

    #include "opencv2/highgui/highgui.hpp"
    #include <opencv2/imgproc/imgproc.hpp>

    #include <iostream>
    #include <string>
    #include "Frame.h"
    #include "TimeDate.h"
    #include "Camera.h"
    #include "arv.h"
    #include "arvinterface.h"
    #include <time.h>
    #include <algorithm>
    #include "EParser.h"

    #define BOOST_LOG_DYN_LINK 1

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

    class CameraLucidArena: public Camera
    {

        private:

            static boost::log::sources::severity_logger< LogSeverityLevel > logger;

            static class Init{

                public:

                    Init(){

                        logger.add_attribute("ClassName", boost::log::attributes::constant<std::string>("CameraLucidArena"));

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
            double          fpsMin;                 // Camera's minimum frame rate.
            double          fpsMax;                 // Camera's maximum frame rate.
            const char*     capsString;
            int             gain;                   // Camera's gain.
            double          exp;                    // Camera's exposure time.
            bool            shiftBitsImage;         // For example : bits are shifted for dmk's frames.
            guint64         nbCompletedBuffers;     // Number of frames successfully received.
            guint64         nbFailures;             // Number of frames failed to be received.
            guint64         nbUnderruns;
            int             frameCounter;           // Counter of success received frames.

        public :

            CameraLucidArena(bool shift);

            CameraLucidArena();

            ~CameraLucidArena();

            std::vector<std::pair<int,std::string>> getCamerasList();

            bool listCameras();

            bool createDevice(int id);

            bool grabInitialization();

            void grabCleanse();

            bool acqStart();

            void acqStop();

            bool grabImage(Frame& newFrame);

            bool grabSingleImage(Frame &frame, int camID);

            bool getDeviceNameById(int id, std::string &device);

            void getExposureBounds(double &eMin, double &eMax);

            void getFPSBounds(double &fMin, double &fMax);

            void getGainBounds(int &gMin, int &gMax);

            bool getPixelFormat(CamPixFmt &format);

            bool getFrameSize(int &x, int &y, int &w, int &h);

            bool getFPS(double &value);

            std::string getModelName();

            double getExposureTime();

            bool setExposureTime(double exp);

            bool setGain(int gain);

            bool setFPS(double fps);

            bool setFrameSize(int startx, int starty, int width, int height, bool customSize);

            bool setPixelFormat(CamPixFmt depth);

            void saveGenicamXml(string p);

            bool setSize(int startx, int starty, int width, int height, bool customSize);

            void getAvailablePixelFormats();

    };

#endif