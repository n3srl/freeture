#pragma once
/*                      CameraV4l2.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                       2018 Chiara Marmo
*                                     GEOPS-UPSUD
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
* \file    CameraV4l2.h
* \author  Yoan Audureau -- Chiara Marmo -- GEOPS-UPSUD
* \version 1.2
* \date    19/03/2019
*/
#include "Commons.h"

#ifdef LINUX

    #include <opencv2/opencv.hpp>

    #include <iostream>
    #include <string>
    #include "Frame.h"
    #include "TimeDate.h"
    #include "Camera.h"
    #include <time.h>

    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <assert.h>


    #include <fcntl.h>              /* low-level i/o */
    #include <unistd.h>
    #include <errno.h>
    #include <sys/stat.h>
    #include <sys/types.h>
    #include <sys/time.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>

    #include <linux/videodev2.h>

    #include "EParser.h"
   
    #include "ELogSeverityLevel.h"
    #include "PixFmtConv.h"
    #include <algorithm>

    namespace freeture
    {
        class CameraDescription;
        class cameraParam;

        class CameraV4l2 : public Camera {

        private:

            const char* mDeviceName;
            int fd;
            double expMin, expMax, exp;
            int gainMin, gainMax, gain;
            int mStartX, mStartY, mWidth, mHeight;
            int mFrameCounter;
            struct v4l2_format mFormat;
            bool mCustomSize;

        public:

            void init_userp(unsigned int buffer_size);
            void init_mmap(void);
            void init_read(unsigned int buffer_size);
            int read_frame(void);
            void errno_exit(const char* s);
            int xioctl(int fh, int request, void* arg);

            CameraV4l2(CameraDescription, cameraParam);

            ~CameraV4l2();

            bool getInfos();

            std::vector<std::pair<int, std::string>> getCamerasList();

            bool listCameras();

            bool createDevice(int id);

            bool setSize(int startx, int starty, int width, int height, bool customSize);

            void grabCleanse();

            bool acqStart();

            void acqStop();

            bool grabImage(Frame& newFrame);

            bool grabSingleImage(Frame& frame);

            bool getDeviceNameById(int id, std::string& device);

            bool getCameraName();

            void getExposureBounds(double& eMin, double& eMax);

            void getGainBounds(int& gMin, int& gMax);

            bool getPixelFormat(CamPixFmt& format);

            bool getFrameSize(int& x, int& y, int& w, int& h);

            bool getFrameSizeEnum();

            bool getFPS(double& value);

            bool getFpsEnum(std::vector<double>& values);

            std::string getModelName();

            double getExposureTime();

            bool setExposureTime(double exp);

            bool setGain(int gain);

            bool setFPS(double fps);

            bool setPixelFormat(CamPixFmt depth);

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

        private:

            bool convertImage(unsigned char* buffer, cv::Mat& image);

            bool setSize();

        };
    }
#endif
