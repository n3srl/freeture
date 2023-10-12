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

#pragma once

#include <math.h>
#include <vector>
#include <iterator>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Frame.h"
#include "LocalEvent.h"
#include "SaveImg.h"
#include "TimeDate.h"

using namespace cv;


class GlobalEvent {

    private :

        int     geAge;
        int     geAgeLastLE;
        TimeDate::Date  geDate;
        Mat     geMap;
        int     geFirstFrameNum;
        int     geLastFrameNum;
        Mat     geDirMap;
        float   geShifting;
        bool    newLeAdded;
        bool    geLinear;
        int     geBadPoint;
        int     geGoodPoint;
        Scalar  geColor;
        Mat     geMapColor;


    public :

        std::vector<LocalEvent>  LEList;
        std::vector<bool>        ptsValidity;
        std::vector<float>       distBtwPts;
        std::vector<float>       distBtwMainPts;
        std::vector<Point>       mainPts;
        std::vector<Point>       pts;
        Point leDir;
        Point geDir;

        std::vector<Point>       listA;
        std::vector<Point>       listB;
        std::vector<Point>       listC;
        std::vector<Point>       listu;
        std::vector<Point>       listv;
        std::vector<float>       listAngle;
        std::vector<float>       listRad;
        std::vector<bool>        mainPtsValidity;
        std::vector<bool>         clusterNegPos;


        GlobalEvent(TimeDate::Date frameDate, int frameNum, int frameHeight, int frameWidth, Scalar c);

        ~GlobalEvent();

        Mat getMapEvent() {return geMap;};
        Mat getDirMap() {return geDirMap;};
        int getAge() {return geAge;};
        int getAgeLastElem() {return geAgeLastLE;};
        TimeDate::Date getDate() {return geDate;};
        bool getLinearStatus() {return geLinear;};
        float getVelocity() {return geShifting;};
        bool getNewLEStatus() {return newLeAdded;};
        int getBadPos() {return geBadPoint;};
        int getGoodPos() {return geGoodPoint;};
        int getNumFirstFrame() {return geFirstFrameNum;};
        int getNumLastFrame() {return geLastFrameNum;};
        Mat getGeMapColor() {return geMapColor;};

        void setAge(int a) {geAge = a;};
        void setAgeLastElem(int a) {geAgeLastLE = a;};
        void setMapEvent(Mat m) {m.copyTo(geMap);};
        void setNewLEStatus(bool s) {newLeAdded = s;};
        void setNumFirstFrame(int n) {geFirstFrameNum = n;};
        void setNumLastFrame(int n) {geLastFrameNum = n;};

        bool ratioFramesDist(std::string &msg);

        bool addLE(LocalEvent le);
        bool continuousGoodPos(int n, std::string &msg);
        bool continuousBadPos(int n);
        bool negPosClusterFilter(std::string &msg);

};
