#pragma once
/*
                            CameraVideo.h

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
*   Last modified:      20/10/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    CameraVideo.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
* \brief   Acquisition thread with video in input.
*/
#include "Commons.h"

#include "Frame.h"
#include "SaveImg.h"
#include "TimeDate.h"
#include "Conversion.h"
#include "Camera.h"
#include <string>
#include <vector>
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
#include <boost/filesystem.hpp>
#include <boost/circular_buffer.hpp>

using namespace boost::filesystem;

namespace freeture
{
    class CameraDescription;
    class cameraParam;

    class CameraVideo : public Camera {

    private:

        static boost::log::sources::severity_logger< LogSeverityLevel > logger;

        static class Init {

        public:

            Init() {

                logger.add_attribute("ClassName", boost::log::attributes::constant<std::string>("CameraVideo"));

            }

        }initializer;

        int                 mFrameWidth;
        int                 mFrameHeight;
        cv::VideoCapture        mCap;
        bool                mReadDataStatus;
        int                 mVideoID;
        std::vector<std::string>      mVideoList;

    public:

        CameraVideo(CameraDescription, cameraParam , std::vector<std::string> videoList, bool verbose);

        ~CameraVideo(void);


        bool acqStart() { return true; };

        bool listCameras() { return true; };

        bool grabImage(Frame& img);


        bool getStopStatus();

        /**
        * Get data status : Is there another video to use in input ?
        *
        * @return If there is still a video to load in input.
        */
        bool getDataSetStatus();

        /**
        * Load next video if there is.
        *
        * @return Success status to load next data set.
        */
        bool loadNextDataSet(std::string& location);

        bool getFPS(double& value) { value = 0; return false; };

        bool setExposureTime(double exp) { return true; };

        bool setGain(int gain) { return true; };

        bool setFPS(double fps) { return true; };

        bool setPixelFormat(CamPixFmt format) { return true; };

        bool setSize(int x, int y, int width, int height, bool customSize) { return true; };

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

        bool createDevice() override;

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
