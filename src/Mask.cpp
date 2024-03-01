/*
                                Mask.cpp

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
*   Last modified:      20/07/2015
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    Mask.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    26/11/2014
*/
#include "Mask.h"

#include <opencv2/imgcodecs.hpp>

#include "Constants.h"

using namespace freeture;
using namespace std;


Mask::Mask(int timeInterval, bool customMask, std::string customMaskPath, bool downsampleMask, CamPixFmt format, bool updateMask):
mUpdateInterval(timeInterval), mUpdateMask(updateMask) {

    mMaskToCreate = false;
    updateStatus = false;
    refDate = to_simple_string(boost::posix_time::second_clock::universal_time());
    satMap = boost::circular_buffer<cv::Mat>(2);

    // Load a mask from file.
    if(customMask) {

        mOriginalMask = imread(customMaskPath, cv::ImreadModes::IMREAD_GRAYSCALE);
        //SaveImg::saveJPEG(mOriginalMask, "/home/mOriginalMask");
        if(!mOriginalMask.data)
            throw "Fail to load the mask from its path.";

        if(downsampleMask)
            pyrDown(mOriginalMask, mOriginalMask, cv::Size(mOriginalMask.cols/2, mOriginalMask.rows/2));

        mOriginalMask.copyTo(mCurrentMask);

    }else{

        mMaskToCreate = true;

    }

    // Estimate saturated value.
    switch(format) {

        case CamPixFmt::MONO12 :

                saturatedValue = 4092;

            break;

        default :

                saturatedValue = 254;

    }

}

bool Mask::applyMask(cv::Mat &currFrame) {

    if(mMaskToCreate) {

        mOriginalMask = cv::Mat(currFrame.rows, currFrame.cols, CV_8UC1, cv::Scalar(255));
        mOriginalMask.copyTo(mCurrentMask);
        mMaskToCreate = false;

    }

    if(mUpdateMask) {

        if(updateStatus) {

            if(mCurrentMask.rows != currFrame.rows || mCurrentMask.cols != currFrame.cols) {

                throw runtime_error("Mask's size is not correct according to frame's size.");

            }

            // Reference date.
            refDate = to_simple_string(boost::posix_time::second_clock::universal_time());

            cv::Mat saturateMap = ImgProcessing::buildSaturatedMap(currFrame, saturatedValue);

            // Dilatation of the saturated map.
            int dilation_size = 10;
            cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2*dilation_size + 1, 2*dilation_size+1), cv::Point(dilation_size, dilation_size));
            cv::dilate(saturateMap, saturateMap, element);

            satMap.push_back(saturateMap);

            if(satMap.size() == 2) {

                cv::Mat temp = satMap.front() & satMap.back();
                bitwise_not(temp,temp);
                temp.copyTo(mCurrentMask, mOriginalMask);

            }

            cv::Mat temp; currFrame.copyTo(temp, mCurrentMask);
            temp.copyTo(currFrame);

            updateStatus = false;
            return true; // Mask not applied, only computed.

        }

        std::string nowDate = to_simple_string(boost::posix_time::second_clock::universal_time());
        boost::posix_time::ptime t1(boost::posix_time::time_from_string(refDate));
        boost::posix_time::ptime t2(boost::posix_time::time_from_string(nowDate));
        boost::posix_time::time_duration td = t2 - t1;
        long diffTime = td.total_seconds();

        if(diffTime >= mUpdateInterval) {

            updateStatus = true;

        }

        if (LOG_SPAM_FRAME_STATUS)
            LOG_DEBUG << "Mask::applyMask;" << "NEXT MASK : " << (mUpdateInterval - (int)diffTime) << "s" << std::endl;

    }

    if(!mCurrentMask.data || (mCurrentMask.rows != currFrame.rows && mCurrentMask.cols != currFrame.cols)) {
        mMaskToCreate = true;
        return true;
    }

    cv::Mat temp; currFrame.copyTo(temp, mCurrentMask);
    temp.copyTo(currFrame);

    return false; // Mask applied.

}

void Mask::resetMask() {

    mOriginalMask.copyTo(mCurrentMask);
    updateStatus = true;
    satMap.clear();

}


