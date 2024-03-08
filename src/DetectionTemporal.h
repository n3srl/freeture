#pragma once
/*
                            DetectionTemporal.h

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
* \file    DetectionTemporal.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
* \brief   Detection method by temporal analysis.
*/

#include "Commons.h"

#include <string>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>

#include "Detection.h"
#include "GlobalEvent.h"
#include "LocalEvent.h"
#include "SParam.h"
#include "TimeDate.h"

// 
// 
// #ifdef LINUX
//     #define BOOST_LOG_DYN_LINK 1
// #endif
// 
// #include <boost/circular_buffer.hpp>
// #include <boost/tokenizer.hpp>
// #include <boost/log/common.hpp>
// #include <boost/log/expressions.hpp>
// #include <boost/log/utility/setup/file.hpp>
// #include <boost/log/utility/setup/console.hpp>
// #include <boost/log/utility/setup/common_attributes.hpp>
// #include <boost/log/attributes/named_scope.hpp>
// #include <boost/log/attributes.hpp>
// #include <boost/log/sinks.hpp>
// #include <boost/log/sources/logger.hpp>
// #include <boost/log/core.hpp>
// #include "ELogSeverityLevel.h"
// #include "TimeDate.h"
// #include "Fits2D.h"
// #include "Fits.h"
// #include "Frame.h"
// #include "ImgProcessing.h"
// #include "EStackMeth.h"
// #include "ECamPixFmt.h"
// #include "GlobalEvent.h"
// #include "LocalEvent.h"
// #include "Detection.h"
// #include "EParser.h"
// #include "SaveImg.h"
// #include <vector>
// #include <utility>
// #include <iterator>
// #include <algorithm>
// #include <boost/filesystem.hpp>
// #include "Mask.h"
// 
// using namespace boost::filesystem;
// namespace logging = boost::log;
// namespace sinks = boost::log::sinks;
// namespace attrs = boost::log::attributes;
// namespace src = boost::log::sources;
// namespace expr = boost::log::expressions;
// namespace keywords = boost::log::keywords;

namespace freeture
{
    class Mask;

    class DetectionTemporal : public Detection {

    private:
        std::vector<std::shared_ptr<GlobalEvent>>  mListGlobalEvents;      // List of global events (Events spread on several frames).
        std::vector<cv::Point>                   mSubdivisionPos;        // Position (origin in top left) of 64 subdivisions.
        std::vector<cv::Scalar>                  mListColors;            // One color per local event.
        cv::Mat                             mLocalMask;             // Mask used to remove isolated white pixels.
        bool                            mSubdivisionStatus;     // If subdivisions positions have been computed.
        cv::Mat                             mPrevThresholdedMap;
        std::shared_ptr<GlobalEvent>   mGeToSave;              // Global event to save.
        int                             mRoiSize[2];
        int                             mImgNum;                // Current frame number.
        cv::Mat                             mPrevFrame;             // Previous frame.
        cv::Mat                             mStaticMask;
        std::string                          mDebugCurrentPath;
        int                             mDataSetCounter;
        bool                            mDebugUpdateMask;
        Mask* mMaskManager;
        std::vector<std::string>                  debugFiles;
        detectionParam                  mdtp;
        cv::VideoWriter                     mVideoDebugAutoMask;

    public:

        DetectionTemporal(detectionParam dp, CamPixFmt fmt);

        ~DetectionTemporal();

        void initMethod(std::string cfgPath) {}

        bool runDetection(std::shared_ptr<Frame> c) override;

        void saveDetectionInfos(std::string p, int nbFramesAround) override;

        void resetDetection(bool loadNewDataSet) override;

        void resetMask() override;

        int getEventFirstFrameNb() override { return (*mGeToSave).getNumFirstFrame(); } ;

        TimeDate::Date getEventDate() override { return (*mGeToSave).getDate(); };

        int getEventLastFrameNb() override { return (*mGeToSave).getNumLastFrame(); };

        std::vector<std::string> getDebugFiles() override;

    private:

        void createDebugDirectories(bool cleanDebugDirectory);

        int selectThreshold(cv::Mat i) { return 0; }

        std::vector<cv::Scalar> getColorInEventMap(cv::Mat& eventMap, cv::Point roiCenter);

        void colorRoiInBlack(cv::Point p, int h, int w, cv::Mat& region);

        void analyseRegion(cv::Mat& subdivision,
            cv::Mat& absDiffBinaryMap,
            cv::Mat& eventMap,
            cv::Mat& posDiff,
            int posDiffThreshold,
            cv::Mat& negDiff,
            int negDiffThreshold,
            std::vector<LocalEvent>& listLE,
            cv::Point subdivisionPos,
            int maxNbLE,
            int numFrame,
            std::string& msg,
            TimeDate::Date cFrameDate);



    };
}
