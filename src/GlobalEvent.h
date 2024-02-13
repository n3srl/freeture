#pragma once
/*
                                GlobalEvent.h

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
* \file    GlobalEvent.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
* \brief   A Detected event occured on different consecutives frames in time.
*/

#include "Commons.h"

#include <math.h>
#include <vector>
#include <iterator>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Frame.h"
#include "LocalEvent.h"
#include "SaveImg.h"
#include "TimeDate.h"

namespace freeture
{

    class GlobalEvent {

    private:

        int     geAge;
        int     geAgeLastLE;
        TimeDate::Date  geDate;
        cv::Mat     geMap;
        int     geFirstFrameNum;
        int     geLastFrameNum;
        cv::Mat     geDirMap;
        float   geShifting;
        bool    newLeAdded;
        bool    geLinear;
        int     geBadPoint;
        int     geGoodPoint;
        cv::Scalar  geColor;
        cv::Mat     geMapColor;


    public:

        std::vector<LocalEvent>  LEList;
        std::vector<bool>        ptsValidity;
        std::vector<float>       distBtwPts;
        std::vector<float>       distBtwMainPts;
        std::vector<cv::Point>       mainPts;
        std::vector<cv::Point>       pts;
        cv::Point leDir;
        cv::Point geDir;

        std::vector<cv::Point>       listA;
        std::vector<cv::Point>       listB;
        std::vector<cv::Point>       listC;
        std::vector<cv::Point>       listu;
        std::vector<cv::Point>       listv;
        std::vector<float>       listAngle;
        std::vector<float>       listRad;
        std::vector<bool>        mainPtsValidity;
        std::vector<bool>         clusterNegPos;


        GlobalEvent(TimeDate::Date frameDate, int frameNum, int frameHeight, int frameWidth, cv::Scalar c);

        ~GlobalEvent();

        cv::Mat getMapEvent() { return geMap; };
        cv::Mat getDirMap() { return geDirMap; };
        int getAge() { return geAge; };
        int getAgeLastElem() { return geAgeLastLE; };
        TimeDate::Date getDate() { return geDate; };
        bool getLinearStatus() { return geLinear; };
        float getVelocity() { return geShifting; };
        bool getNewLEStatus() { return newLeAdded; };
        int getBadPos() { return geBadPoint; };
        int getGoodPos() { return geGoodPoint; };
        int getNumFirstFrame() { return geFirstFrameNum; };
        int getNumLastFrame() { return geLastFrameNum; };
        cv::Mat getGeMapColor() { return geMapColor; };

        void setAge(int a) { geAge = a; };
        void setAgeLastElem(int a) { geAgeLastLE = a; };
        void setMapEvent(cv::Mat m) { m.copyTo(geMap); };
        void setNewLEStatus(bool s) { newLeAdded = s; };
        void setNumFirstFrame(int n) { geFirstFrameNum = n; };
        void setNumLastFrame(int n) { geLastFrameNum = n; };

        bool ratioFramesDist(std::string& msg);

        bool addLE(LocalEvent le);
        bool continuousGoodPos(int n, std::string& msg);
        bool continuousBadPos(int n);
        bool negPosClusterFilter(std::string& msg);

    };
}
