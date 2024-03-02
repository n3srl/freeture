/*
                    DetectionTemporal.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                       2014-2018 Chiara Marmo
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
*   Last modified:      12/03/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    DetectionTemporal.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    12/03/2018
* \brief   Detection method by temporal movement.
*/

#include "DetectionTemporal.h"

#include "Logger.h"

#include <boost/filesystem.hpp>

#include "Mask.h"
#include "ImgProcessing.h"
#include "SaveImg.h"

namespace fs = boost::filesystem;
using namespace freeture;
using namespace std;

DetectionTemporal::DetectionTemporal(detectionParam dtp, CamPixFmt fmt) {

    mListColors.push_back(cv::Scalar(0,0,139));      // DarkRed
    mListColors.push_back(cv::Scalar(0,0,255));      // Red
    mListColors.push_back(cv::Scalar(0,100,100));    // IndianRed
    mListColors.push_back(cv::Scalar(92,92,205));    // Salmon
    mListColors.push_back(cv::Scalar(0,140,255));    // DarkOrange
    mListColors.push_back(cv::Scalar(30,105,210));   // Chocolate
    mListColors.push_back(cv::Scalar(0,255,255));    // Yellow
    mListColors.push_back(cv::Scalar(140,230,240));  // Khaki
    mListColors.push_back(cv::Scalar(224,255,255));  // LightYellow
    mListColors.push_back(cv::Scalar(211,0,148));    // DarkViolet
    mListColors.push_back(cv::Scalar(147,20,255));   // DeepPink
    mListColors.push_back(cv::Scalar(255,0,255));    // Magenta
    mListColors.push_back(cv::Scalar(0,100,0));      // DarkGreen
    mListColors.push_back(cv::Scalar(0,128,128));    // Olive
    mListColors.push_back(cv::Scalar(0,255,0));      // Lime
    mListColors.push_back(cv::Scalar(212,255,127));  // Aquamarine
    mListColors.push_back(cv::Scalar(208,224,64));   // Turquoise
    mListColors.push_back(cv::Scalar(205,0,0));      // Blue
    mListColors.push_back(cv::Scalar(255,191,0));    // DeepSkyBlue
    mListColors.push_back(cv::Scalar(255,255,0));    // Cyan

    mImgNum = 0;
    mDebugUpdateMask = false;
    mSubdivisionStatus = false;
    mDataSetCounter = 0;
    mRoiSize[0] = 10;
    mRoiSize[1] = 10;
    mdtp = dtp;

    mMaskManager = new Mask(dtp.DET_UPDATE_MASK_FREQUENCY, dtp.ACQ_MASK_ENABLED, dtp.ACQ_MASK_PATH, dtp.DET_DOWNSAMPLE_ENABLED, fmt, dtp.DET_UPDATE_MASK);

    // Create local mask to eliminate single white pixels.
    cv::Mat maskTemp(3,3,CV_8UC1, cv::Scalar(255));
    maskTemp.at<uchar>(1, 1) = 0;
    maskTemp.copyTo(mLocalMask);

    mdtp.DET_DEBUG_PATH = mdtp.DET_DEBUG_PATH + "/";

    mDebugCurrentPath = mdtp.DET_DEBUG_PATH;

    // Create directories for debugging method.
    if(dtp.DET_DEBUG)
        createDebugDirectories(true);

}

DetectionTemporal::~DetectionTemporal() {

    if(mMaskManager != NULL)
        delete mMaskManager;

}

void DetectionTemporal::resetDetection(bool loadNewDataSet){

    LOG_DEBUG << "Clear global events list." << endl;
    mListGlobalEvents.clear();
    // Clear list of files to send by mail.
    debugFiles.clear();
    mSubdivisionStatus = false;
    mPrevThresholdedMap.release();
    mPrevFrame.release();

    if(mdtp.DET_DEBUG && loadNewDataSet) {
        mDataSetCounter++;
        createDebugDirectories(false);
    }
}

void DetectionTemporal::resetMask(){
    LOG_DEBUG << "DetectionTemporal::resetMask" << endl;
    mMaskManager->resetMask();

}

void DetectionTemporal::createDebugDirectories(bool cleanDebugDirectory){
    LOG_DEBUG << "DetectionTemporal::createDebugDirectories" << endl;
    mDebugCurrentPath = mdtp.DET_DEBUG_PATH + "debug_" + Conversion::intToString(mDataSetCounter) + "/" ;

    if(cleanDebugDirectory) {

        const fs::path p0 = fs::path(mdtp.DET_DEBUG_PATH);

        if(fs::exists(p0)) {
            fs::remove_all(p0);
        }else {
            fs::create_directories(p0);
        }

    }

    const fs::path p1 = fs::path(mDebugCurrentPath);

    if(!fs::exists(p1))
        fs::create_directories(p1);

    vector<string> debugSubDir;
    debugSubDir.push_back("original");
    debugSubDir.push_back("absolute_difference");
    debugSubDir.push_back("event_map_initial");
    debugSubDir.push_back("event_map_filtered");
    debugSubDir.push_back("absolute_difference_dilated");
    debugSubDir.push_back("neg_difference_thresholded");
    debugSubDir.push_back("pos_difference_thresholded");
    debugSubDir.push_back("neg_difference");
    debugSubDir.push_back("pos_difference");

    for(int i = 0; i< debugSubDir.size(); i++){

        const fs::path path(mDebugCurrentPath + debugSubDir.at(i));

        if(!fs::exists(path)) {
            fs::create_directories(path);
        }

    }

}

void DetectionTemporal::saveDetectionInfos(string p, int nbFramesAround){
    LOG_DEBUG << "DetectionTemporal::saveDetectionInfos" << endl;
    // Save ge map.
    if(mdtp.temporal.DET_SAVE_GEMAP) {
        SaveImg::saveBMP((*mGeToSave).getMapEvent(), p + "GeMap");
        debugFiles.push_back("GeMap.bmp");
    }

    // Save dir map.
    if(mdtp.temporal.DET_SAVE_DIRMAP) {
        SaveImg::saveBMP((*mGeToSave).getDirMap(), p + "DirMap");
    }

    // Save infos.
    /*if(mdtp.temporal.DET_SAVE_GE_INFOS) {

        ofstream infFile;
        string infFilePath = p + "GeInfos.txt";
        infFile.open(infFilePath.c_str());

        infFile << " * AGE              : " << (*mGeToSave).getAge()             << "\n";
        infFile << " * AGE LAST ELEM    : " << (*mGeToSave).getAgeLastElem()     << "\n";
        infFile << " * LINEAR STATE     : " << (*mGeToSave).getLinearStatus()    << "\n";
        infFile << " * BAD POS          : " << (*mGeToSave).getBadPos()          << "\n";
        infFile << " * GOOD POS         : " << (*mGeToSave).getGoodPos()         << "\n";
        infFile << " * NUM FIRST FRAME  : " << (*mGeToSave).getNumFirstFrame()   << "\n";
        infFile << " * NUM LAST FRAME   : " << (*mGeToSave).getNumLastFrame()    << "\n";

        float d = sqrt(pow((*mGeToSave).mainPts.back().x - (*mGeToSave).mainPts.front().x,2.0) + pow((*mGeToSave).mainPts.back().y - (*mGeToSave).mainPts.front().y,2.0));
        infFile << "\n * Distance between first and last  : " << d << "\n";

        infFile << "\n * MainPoints position : \n";
        for(int i = 0; i < (*mGeToSave).mainPts.size(); i++)
            infFile << "    (" << (*mGeToSave).mainPts.at(i).x << ";"<<  (*mGeToSave).mainPts.at(i).y << ")\n";

        infFile << "\n * MainPoints details : \n";
        for(int i = 0; i < (*mGeToSave).listA.size(); i++){

            infFile << "    A(" << (*mGeToSave).listA.at(i).x << ";" << (*mGeToSave).listA.at(i).y << ") ----> ";
            infFile << "    B(" << (*mGeToSave).listB.at(i).x << ";" << (*mGeToSave).listB.at(i).y << ") ----> ";
            infFile << "    C(" << (*mGeToSave).listC.at(i).x << ";" << (*mGeToSave).listC.at(i).y << ")\n";
            infFile << "    u(" << (*mGeToSave).listu.at(i).x << ";" << (*mGeToSave).listu.at(i).y << ")       ";
            infFile << "    v(" << (*mGeToSave).listv.at(i).x << ";" << (*mGeToSave).listv.at(i).y << ")\n";
            infFile << "    Angle rad between BA' / BC = " << (*mGeToSave).listRad.at(i) << "\n";
            infFile << "    Angle between BA' / BC = " << (*mGeToSave).listAngle.at(i) << "\n";

            if((*mGeToSave).mainPtsValidity.at(i)) infFile << "    NEW POSITION ACCEPTED\n\n";
            else infFile << "    NEW POSITION REFUSED\n\n";

        }

        infFile.close();
    }*/

    // Save positions.
    if(mdtp.temporal.DET_SAVE_POS) {

        std::ofstream posFile;
        string posFilePath = p + "positions.txt";
        posFile.open(posFilePath.c_str());

        // Number of the first frame associated to the event.
        int numFirstFrame = -1;

        vector<LocalEvent>::iterator itLe;
        for(itLe = (*mGeToSave).LEList.begin(); itLe!=(*mGeToSave).LEList.end(); ++itLe) {

            if(numFirstFrame == -1)
                numFirstFrame = (*itLe).getNumFrame();

            cv::Point pos = (*itLe).getMassCenter();

            int positionY = 0;
            if(mdtp.DET_DOWNSAMPLE_ENABLED) {
                pos*=2;
                positionY = mPrevFrame.rows*2 - pos.y;
            }else {
                positionY = mPrevFrame.rows - pos.y;
            }

            // NUM_FRAME    POSITIONX     POSITIONY (inversed)
            string line = Conversion::intToString((*itLe).getNumFrame() - numFirstFrame + nbFramesAround) + "               (" + Conversion::intToString(pos.x)  + ";" + Conversion::intToString(positionY) + ")                 " + TimeDate::getIsoExtendedFormatDate((*itLe).mFrameAcqDate)+ "\n";
            posFile << line;

        }

        posFile.close();

    }
}

vector<string> DetectionTemporal::getDebugFiles() {
    LOG_DEBUG << "DetectionTemporal::getDebugFiles" << endl;
    return debugFiles;

}

bool DetectionTemporal::runDetection(std::shared_ptr<Frame> c) {
    if (LOG_SPAM_FRAME_STATUS)
        LOG_DEBUG << "DetectionTemporal::runDetection" << endl;

    if(!mSubdivisionStatus) {
        if (LOG_SPAM_FRAME_STATUS)
            LOG_DEBUG << "DetThread run DetectionTemporal !mSubDivisionStatus" << endl;
        mSubdivisionPos.clear();

        int h = c->Image->rows;
        int w = c->Image->cols;

        if(mdtp.DET_DOWNSAMPLE_ENABLED) {
            h /= 2;
            w /= 2;
        }

        ImgProcessing::subdivideFrame(mSubdivisionPos, 8, h, w);
        mSubdivisionStatus = true;

        if(mdtp.DET_DEBUG) {

            cv::Mat s = cv::Mat(h, w,CV_8UC1, cv::Scalar(0));

            for(int i = 0; i < 8; i++) {
               line(s, cv::Point(0, i * (h/8)), cv::Point(w - 1, i * (h/8)), cv::Scalar(255), 1);
               line(s, cv::Point(i * (w/8), 0), cv::Point(i * (w/8), h-1), cv::Scalar(255), 1);
            }

            SaveImg::saveBMP(s, mDebugCurrentPath + "subdivisions_map");

        }

    } else {
        if (LOG_SPAM_FRAME_STATUS)
            LOG_DEBUG << "DetThread run DetectionTemporal mSubDivisionStatus" << endl;

        double tDownsample  = 0;
        double tAbsDiff     = 0;
        double tPosDiff     = 0;
        double tNegDiff     = 0;
        double tDilate      = 0;
        double tThreshold   = 0;
        double tStep1       = 0;
        double tStep2       = 0;
        double tStep3       = 0;
        double tStep4       = 0;
        double tTotal       = (double)cv::getTickCount();

        /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        /// %%%%%%%%%%%%%%%%%%%%%%%%%%% STEP 1 : FILETRING / THRESHOLDING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        double tstep1 = (double)cv::getTickCount();
        cv::Mat currImg;

        // ------------------------------
        //    Downsample current frame.
        // -------------------------------

        if(mdtp.DET_DOWNSAMPLE_ENABLED) {

            tDownsample = (double)cv::getTickCount();
            pyrDown(*c->Image.get(), currImg, cv::Size(c->Image->cols / 2, c->Image->rows / 2));
            tDownsample = ((double)cv::getTickCount() - tDownsample);

        }else {

            c->Image->copyTo(currImg);

        }

        // Apply mask on currImg.
        // If true is returned, it means that the mask has been updated and applied on currImg. Detection process can't continue.
        // If false is returned, it means that the mask has not been updated. Detection process can continue.
        if(!mMaskManager->applyMask(currImg)) {

            // --------------------------------
            //      Check previous frame.
            // --------------------------------

            if(!mPrevFrame.data) {

                currImg.copyTo(mPrevFrame);
                return false;

            }

            // --------------------------------
            //          Differences.
            // --------------------------------

            cv::Mat absdiffImg, posDiffImg, negDiffImg;

            // Absolute difference.
            tAbsDiff = (double)cv::getTickCount();
            cv::absdiff(currImg, mPrevFrame, absdiffImg);
            tAbsDiff = (double)cv::getTickCount() - tAbsDiff;

            // Positive difference.
            tPosDiff = (double)cv::getTickCount();
            cv::subtract(currImg,mPrevFrame,posDiffImg,mMaskManager->mCurrentMask);
            tPosDiff = (double)cv::getTickCount() - tPosDiff;

            // Negative difference.
            tNegDiff = (double)cv::getTickCount();
            cv::subtract(mPrevFrame,currImg,negDiffImg,mMaskManager->mCurrentMask);
            tNegDiff = (double)cv::getTickCount() - tNegDiff;

            // ---------------------------------
            //  Dilatate absolute difference.
            // ---------------------------------

            tDilate = (double)cv::getTickCount();
            int dilation_size = 2;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*dilation_size + 1, 2*dilation_size+1), cv::Point(dilation_size, dilation_size));
            cv::dilate(absdiffImg, absdiffImg, element);
            tDilate = (double)cv::getTickCount() - tDilate;

            // ------------------------------------------------------------------------------
            //   Threshold absolute difference / positive difference / negative difference
            // ------------------------------------------------------------------------------

            tThreshold = (double)cv::getTickCount();
            cv::Mat absDiffBinaryMap = ImgProcessing::thresholding(absdiffImg, mMaskManager->mCurrentMask, 3, Thresh::MEAN);
            tThreshold = (double)cv::getTickCount() - tThreshold;

            cv::Scalar meanPosDiff, stddevPosDiff, meanNegDiff, stddevNegDiff;
            meanStdDev(posDiffImg, meanPosDiff, stddevPosDiff, mMaskManager->mCurrentMask);
            meanStdDev(negDiffImg, meanNegDiff, stddevNegDiff, mMaskManager->mCurrentMask);
            double posThreshold = stddevPosDiff[0] * 5.0 + 10.0;
            double negThreshold = stddevNegDiff[0] * 5.0 + 10.0;

            if(mdtp.DET_DEBUG) {

                cv::Mat posBinaryMap = ImgProcessing::thresholding(posDiffImg, mMaskManager->mCurrentMask, 5, Thresh::STDEV);
                cv::Mat negBinaryMap = ImgProcessing::thresholding(negDiffImg, mMaskManager->mCurrentMask, 5, Thresh::STDEV);

                SaveImg::saveBMP(Conversion::convertTo8UC1(currImg), mDebugCurrentPath + "/original/frame_" + Conversion::intToString(c->mFrameNumber));
                SaveImg::saveBMP(posBinaryMap, mDebugCurrentPath + "/pos_difference_thresholded/frame_" + Conversion::intToString(c->mFrameNumber));
                SaveImg::saveBMP(negBinaryMap, mDebugCurrentPath + "/neg_difference_thresholded/frame_" + Conversion::intToString(c->mFrameNumber));
                SaveImg::saveBMP(absDiffBinaryMap, mDebugCurrentPath + "/absolute_difference_thresholded/frame_" + Conversion::intToString(c->mFrameNumber));
                SaveImg::saveBMP(absdiffImg, mDebugCurrentPath + "/absolute_difference/frame_"+Conversion::intToString(c->mFrameNumber));
                SaveImg::saveBMP(Conversion::convertTo8UC1(posDiffImg), mDebugCurrentPath + "/pos_difference/frame_" + Conversion::intToString(c->mFrameNumber));
                SaveImg::saveBMP(Conversion::convertTo8UC1(negDiffImg), mDebugCurrentPath + "/neg_difference/frame_" + Conversion::intToString(c->mFrameNumber));

            }

            // Current frame is stored as the previous frame.
            currImg.copyTo(mPrevFrame);

            tStep1 = (double)cv::getTickCount() - tStep1;

            /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STEP 2 : FIND LOCAL EVENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            // SUMMARY :
            // Loop binarized absolute difference image.
            // For each white pixel, define a Region of interest (ROI) of 10x10 centered in this pixel.
            // Create a new Local Event initialized with this first ROI or attach this ROI to an existing Local Event.
            // Loop the ROI in the binarized absolute difference image to store position of white pixels.
            // Loop the ROI in the positive difference image to store positions of white pixels.
            // Loop the ROI in the negative difference image to store positions of white pixels.
            // Once the list of Local Event has been completed :
            // Analyze each local event in order to check that pixels can be clearly split in two groups (negative, positive).

            vector <LocalEvent> listLocalEvents;
            vector<LocalEvent>::iterator itLE;
            tStep2 = (double)cv::getTickCount();

            // Event map for the current frame.
            cv::Mat eventMap = cv::Mat(currImg.rows,currImg.cols, CV_8UC3, cv::Scalar(0,0,0));

            // ----------------------------------
            //        Search local events.
            // ----------------------------------

            // Iterator on list of sub-regions.
            vector<cv::Point>::iterator itR;

            for(itR = mSubdivisionPos.begin(); itR != mSubdivisionPos.end(); ++itR) {

                // Extract subdivision from binary map.
                cv::Mat subdivision = absDiffBinaryMap(cv::Rect((*itR).x, (*itR).y, absDiffBinaryMap.cols/8, absDiffBinaryMap.rows/8));

                // Check if there is white pixels.
                if(countNonZero(subdivision) > 0){

                    string debugMsg = "";
                    analyseRegion(  subdivision,
                                    absDiffBinaryMap,
                                    eventMap,
                                    posDiffImg,
                                    posThreshold,
                                    negDiffImg,
                                    negThreshold,
                                    listLocalEvents,
                                    (*itR),
                                    mdtp.temporal.DET_LE_MAX,
                                    c->mFrameNumber,
                                    debugMsg,
                                    c->mDate);
                }
            }

            for(int i = 0; i < listLocalEvents.size(); i++)
                listLocalEvents.at(i).setLeIndex(i);

            if(mdtp.DET_DEBUG) SaveImg::saveBMP(eventMap, mDebugCurrentPath + "/event_map_initial/frame_" + Conversion::intToString(c->mFrameNumber));

            // ----------------------------------
            //         Link between LE.
            // ----------------------------------

            size_t leNumber = listLocalEvents.size();

            // Iterator list on localEvent list : localEvent contains a positive or negative cluster.
            vector<vector<LocalEvent>::iterator > itLePos, itLeNeg;

            // Association of a positive cluster localEvent with a negative cluster localEvent.
            vector<pair<vector<LocalEvent>::iterator, vector<LocalEvent>::iterator> > itPair;

            itLE = listLocalEvents.begin();

            // Search pos and neg alone.
            while(itLE != listLocalEvents.end()) {

                // Le has pos cluster but no neg cluster.
                if((*itLE).getPosClusterStatus() && !(*itLE).getNegClusterStatus()) {
                    itLePos.push_back(itLE);
                }else if(!(*itLE).getPosClusterStatus() && (*itLE).getNegClusterStatus()){
                    itLeNeg.push_back(itLE);
                }

                ++itLE;

            }

            int maxRadius = 50;

            // Try to link a positive cluster to a negative one.
            for(int i = 0; i < itLePos.size(); i++) {

                int nbPotentialNeg = 0;

                vector<LocalEvent>::iterator itChoose;
                vector<vector<LocalEvent>::iterator >::iterator c;

                for(vector<vector<LocalEvent>::iterator >::iterator j = itLeNeg.begin(); j != itLeNeg.end();) {

                    cv::Point A = (*itLePos.at(i)).getMassCenter();
                    cv::Point B = (*(*j)).getMassCenter();
                    double dist = sqrt(pow((A.x - B.x),2) + pow((A.y - B.y),2));

                    if(dist < 50) {

                        nbPotentialNeg++;
                        itChoose = (*j);
                        c = j;

                    }

                    ++j;
                }

                if(nbPotentialNeg == 1) {

                    (*itLePos.at(i)).mergeWithAnOtherLE((*itChoose));
                    (*itLePos.at(i)).setMergedStatus(true);
                    (*itChoose).setMergedStatus(true);
                    itLeNeg.erase(c);

                }

            }

            // Delete pos cluster not merged and negative cluster not merged.
             itLE = listLocalEvents.begin();

            // Search pos and neg alone.
            while(itLE != listLocalEvents.end()) {

                // Le has pos cluster but no neg cluster.
                if(// ((*itLE).getPosClusterStatus() && !(*itLE).getNegClusterStatus() && !(*itLE).getMergedStatus())||
                    (!(*itLE).getPosClusterStatus() && (*itLE).getNegClusterStatus()&& (*itLE).getMergedStatus())) {
                    itLE = listLocalEvents.erase(itLE);
                }else {
                    ++itLE;
                }
            }

            // -----------------------------------
            //            Circle TEST.
            // -----------------------------------

            leNumber = listLocalEvents.size();

            itLE = listLocalEvents.begin();

            while(itLE != listLocalEvents.end()) {

                if((*itLE).getPosClusterStatus() && (*itLE).getNegClusterStatus()) {

                    if((*itLE).localEventIsValid()) {
                        ++itLE;
                    }else {
                        itLE = listLocalEvents.erase(itLE);
                    }

                }else {
                    ++itLE;
                }
            }

            if(mdtp.DET_DEBUG) {

                cv::Mat eventMapFiltered = cv::Mat(currImg.rows,currImg.cols, CV_8UC3, cv::Scalar(0,0,0));

                for(int i = 0; i < listLocalEvents.size(); i++) {

                    cv::Mat roiF(10, 10, CV_8UC3, listLocalEvents.at(i).getColor());

                    for(int j = 0; j < listLocalEvents.at(i).mLeRoiList.size();j++) {
                        if( listLocalEvents.at(i).mLeRoiList.at(j).x-5 > 0 &&
                            listLocalEvents.at(i).mLeRoiList.at(j).x+5 < eventMapFiltered.cols &&
                            listLocalEvents.at(i).mLeRoiList.at(j).y-5 > 0 &&
                            listLocalEvents.at(i).mLeRoiList.at(j).y+5 < eventMapFiltered.rows) {
                            roiF.copyTo(eventMapFiltered(cv::Rect(listLocalEvents.at(i).mLeRoiList.at(j).x - 5, listLocalEvents.at(i).mLeRoiList.at(j).y - 5, 10, 10)));
                        }
                    }

                }

                SaveImg::saveBMP(eventMapFiltered, mDebugCurrentPath + "/event_map_filtered/frame_" + Conversion::intToString(c->mFrameNumber));

            }

            tStep2 = (double)cv::getTickCount() - tStep2;

            /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            /// %%%%%%%%%%%%%%%%%%%%%%%%%% STEP 3 : ATTACH LE TO GE OR CREATE NEW ONE %%%%%%%%%%%%%%%%%%%%%%%%%
            /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            // SUMMARY :
            // Loop list of local events.
            // Create a new global event initialized with the current Local event or attach it to an existing global event.
            // If attached, check the positive-negative couple of the global event.

            // Iterator on list of global event.
            vector<shared_ptr<GlobalEvent>>::iterator itGE;

            tStep3 = (double)cv::getTickCount();

            itLE = listLocalEvents.begin();

            while(itLE != listLocalEvents.end()) {

                bool LELinked = false;
                vector<shared_ptr<GlobalEvent>>::iterator itGESelected;
                bool GESelected = false;

                (*itLE).setNumFrame(c->mFrameNumber);

                for(itGE = mListGlobalEvents.begin(); itGE != mListGlobalEvents.end(); ++itGE){
                    shared_ptr<GlobalEvent> global_event = (*itGE);

                    cv::Mat res = (*itLE).getMap() & global_event->getMapEvent();

                    if(cv::countNonZero(res) > 0){

                        LELinked = true;

                        // The current LE has found a possible global event.
                        if(GESelected){

                            //LOG_DEBUG << "The current LE has found a possible global event."<< endl;

                            // Choose the older global event.
                            if(global_event->getAge() > (*itGESelected)->getAge()){

                                //LOG_DEBUG << "Choose the older global event."<< endl;
                                itGESelected = itGE;
                            }

                        }else{

                            //LOG_DEBUG << "Keep same"<< endl;
                            itGESelected = itGE;
                            GESelected = true;

                        }

                        break;

                    }
                }

                // Add current LE to an existing GE
                if(GESelected){

                    //LOG_DEBUG << "Add current LE to an existing GE ... "<< endl;
                    // Add LE.
                    (*itGESelected)->addLE((*itLE));
                    //LOG_DEBUG << "Flag to indicate that a local event has been added ... "<< endl;
                    // Flag to indicate that a local event has been added.
                    (*itGESelected)->setNewLEStatus(true);
                    //LOG_DEBUG << "reset age of the last local event received by the global event.... "<< endl;
                    // reset age of the last local event received by the global event.
                    (*itGESelected)->setAgeLastElem(0);

                }else{

                    // The current LE has not been linked. It became a new GE.
                    if(mListGlobalEvents.size() < mdtp.temporal.DET_GE_MAX) {

                        //LOG_DEBUG << "Selecting last available color ... "<< endl;
                        cv::Scalar geColor = cv::Scalar(255,255,255);//availableGeColor.back();
                        //LOG_DEBUG << "Deleting last available color ... "<< endl;
                        //availableGeColor.pop_back();
                        //LOG_DEBUG << "Creating new GE ... "<< endl;
                        shared_ptr<GlobalEvent> newGE = make_shared<GlobalEvent>(c->mDate, c->mFrameNumber, currImg.rows, currImg.cols, geColor);
                        //LOG_DEBUG << "Adding current LE ... "<< endl;
                        newGE->addLE((*itLE));
                        //LOG_DEBUG << "Pushing new LE to GE list  ... "<< endl;
                        //Add the new globalEvent to the globalEvent's list
                        mListGlobalEvents.push_back(newGE);

                    }
                }

                itLE = listLocalEvents.erase(itLE); // Delete the current localEvent.

            }

            tStep3 = (double)cv::getTickCount() - tStep3;

            /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STEP 4 : MANAGE LIST GLOBAL EVENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            /// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            tStep4 = (double)cv::getTickCount();    // Count process time of step 4.
            itGE = mListGlobalEvents.begin();   // Iterator on global event list.
            bool saveSignal = false;            // Returned signal to indicate to save a GE or not.

            // Loop global event list to check their characteristics.
            while(itGE != mListGlobalEvents.end()) {
                shared_ptr<GlobalEvent> global_event = (*itGE);

                global_event->setAge(global_event->getAge() + 1); // Increment age.

                // If the current global event has not received any new local event.
                if(!global_event->getNewLEStatus()){

                    global_event->setAgeLastElem(global_event->getAgeLastElem()+1);  // Increment its "Without any new local event age"

                }else{

                    global_event->setNumLastFrame(c->mFrameNumber);
                    global_event->setNewLEStatus(false);
                }

                string msgGe = "";

                // CASE 1 : FINISHED EVENT.
                if(global_event->getAgeLastElem() > 5) 
                {

                    // Linear profil ? Minimum duration respected ?
                    if(global_event->LEList.size() >= 5
                        && global_event->continuousGoodPos(4, msgGe)
                        && global_event->ratioFramesDist(msgGe)
                        && global_event->negPosClusterFilter(msgGe)){

                        mGeToSave = global_event;
                        saveSignal = true;
                        break;

                    } else {
                        itGE = mListGlobalEvents.erase(itGE); // Delete the event.
                       
                    }

                // CASE 2 : NOT FINISHED EVENT.
                }
                else 
                {
                    int nbsec = TimeDate::secBetweenTwoDates(global_event->getDate(), c->mDate);
                    bool maxtime = false;
                    if(nbsec > mdtp.DET_TIME_MAX)
                        maxtime = true;

                    // Check some characteristics : Too long event ? not linear ?
                    if( maxtime
                        || (!global_event->getLinearStatus()
                        && !global_event->continuousGoodPos(5,msgGe))
                        || (!global_event->getLinearStatus()
                        && global_event->continuousBadPos((int)global_event->getAge()/2))){

                        if (!global_event->getLinearStatus() && !global_event->continuousGoodPos(5, msgGe))
                            LOG_DEBUG << "Event discarded because not linear even if good positioned" << endl;
                       
                        if (!global_event->getLinearStatus() && global_event->continuousBadPos((int)global_event->getAge() / 2))
                            LOG_DEBUG << "Event discarded because not linear and bad positioned" << endl;

                        if(maxtime) {
                                TimeDate::Date gedate = global_event->getDate();
                                LOG_INFO << "# GE deleted because max time reached : " << endl;
                                string m = "- global_event.getDate() : "
                                    + Conversion::numbering(4, gedate.year) + Conversion::intToString(gedate.year)
                                    + Conversion::numbering(2, gedate.month) + Conversion::intToString(gedate.month)
                                    + Conversion::numbering(2, gedate.day) + Conversion::intToString(gedate.day) + "T"
                                    + Conversion::numbering(2, gedate.hours) + Conversion::intToString(gedate.hours)
                                    + Conversion::numbering(2, gedate.minutes) + Conversion::intToString(gedate.minutes)
                                    + Conversion::numbering(2, gedate.seconds) + Conversion::intToString((int)gedate.seconds);

                                LOG_INFO << m << endl;

                                LOG_INFO << "- c.mDate : "
                                    << Conversion::numbering(4, c->mDate.year) << Conversion::intToString(c->mDate.year)
                                    << Conversion::numbering(2, c->mDate.month) << Conversion::intToString(c->mDate.month)
                                    << Conversion::numbering(2, c->mDate.day) << Conversion::intToString(c->mDate.day) << "T"
                                    << Conversion::numbering(2, c->mDate.hours) << Conversion::intToString(c->mDate.hours)
                                    << Conversion::numbering(2, c->mDate.minutes) << Conversion::intToString(c->mDate.minutes)
                                    << Conversion::numbering(2, c->mDate.seconds) << Conversion::intToString((int)c->mDate.seconds)
                                    << endl;

                                LOG_INFO << "- difftime in sec : " << nbsec << endl;
                                LOG_INFO << "- maxtime in sec : " << mdtp.DET_TIME_MAX << endl;
                        }

                        itGE = mListGlobalEvents.erase(itGE); // Delete the event.

                    // Let the GE alive.
                    }else if(c->mFrameRemaining < 10 && c->mFrameRemaining != 0) {

                        if(global_event->LEList.size() >= 5 && global_event->continuousGoodPos(4,msgGe) && global_event->ratioFramesDist(msgGe)&& global_event->negPosClusterFilter(msgGe)){

                            mGeToSave = global_event;
                            saveSignal = true;
                            break;

                        } else{
                            itGE = mListGlobalEvents.erase(itGE); // Delete the event.
                        }

                    } else {
                        ++itGE; // Do nothing to the current GE, check the following one.
                    }
                }
            }

            tStep4 = (double)cv::getTickCount() - tStep4;
            tTotal = (double)cv::getTickCount() - tTotal;

            return saveSignal;

        // The mask has been updated.
        } else {

            if(mdtp.DET_DEBUG_UPDATE_MASK) {
                const fs::path p = path(mdtp.DET_DEBUG_PATH + "/mask/");
                if(!fs::exists(p)) fs::create_directories(p);
                SaveImg::saveJPEG(Conversion::convertTo8UC1(currImg), mdtp.DET_DEBUG_PATH + "/mask/umask_" + Conversion::numbering(10,c->mFrameNumber) + Conversion::intToString(c->mFrameNumber));
            }

            currImg.copyTo(mPrevFrame);
        }
    }

    return false;

}

vector<cv::Scalar> DetectionTemporal::getColorInEventMap(cv::Mat &eventMap, cv::Point roiCenter) {

    // ROI in the eventMap.
    cv::Mat roi;

    // ROI extraction from the eventmap.
    eventMap(cv::Rect(roiCenter.x-mRoiSize[0]/2, roiCenter.y-mRoiSize[1]/2, mRoiSize[0], mRoiSize[1])).copyTo(roi);

    unsigned char *ptr = (unsigned char*)roi.data;

    int cn = roi.channels();

    vector<cv::Scalar> listColor;

    bool exist = false;

    for(int i = 0; i < roi.rows; i++){

        for(int j = 0; j < roi.cols; j++){

            cv::Scalar bgrPixel;
            bgrPixel.val[0] = ptr[i*roi.cols*cn + j*cn + 0]; // B
            bgrPixel.val[1] = ptr[i*roi.cols*cn + j*cn + 1]; // G
            bgrPixel.val[2] = ptr[i*roi.cols*cn + j*cn + 2]; // R

            if(bgrPixel.val[0] != 0 || bgrPixel.val[1] != 0 || bgrPixel.val[2] != 0){

                for(int k = 0; k < listColor.size(); k++){

                    if(bgrPixel == listColor.at(k)){

                        exist = true;
                        break;

                    }
                }

                if(!exist)
                    listColor.push_back(bgrPixel);

                exist = false;

            }
        }
    }

    return listColor;

}

void DetectionTemporal::colorRoiInBlack(cv::Point p, int h, int w, cv::Mat &region){

    int posX = p.x - w;
    int posY = p.y - h;

    if(p.x - w < 0) {

        w = p.x + w/2;
        posX = 0;

    }else if(p.x + w/2 > region.cols) {

        w = region.cols - p.x + w/2;

    }

    if(p.y - h < 0) {

        h = p.y + h/2;
        posY = 0;


    }else if(p.y + h/2 > region.rows) {

        h = region.rows - p.y + h/2;

    }

    // Color roi in black in the current region.
    cv::Mat roiBlackRegion(h, w, CV_8UC1, cv::Scalar(0));
    roiBlackRegion.copyTo(region(cv::Rect(posX, posY, w, h)));

}

void DetectionTemporal::analyseRegion(cv::Mat &subdivision,
    cv::Mat& absDiffBinaryMap,
    cv::Mat& eventMap,
    cv::Mat& posDiff,
                                    int posDiffThreshold,
                    cv::Mat &negDiff,
                                    int negDiffThreshold,
                                    vector<LocalEvent> &listLE,
    cv::Point subdivisionPos,     // Origin position of a region in frame (corner top left)
                                    int maxNbLE,
                                    int numFrame,
                                    string &msg,
                                    TimeDate::Date cFrameDate){

int situation = 0;
int nbCreatedLE = 0;
int nbRoiAttachedToLE = 0;
int nbNoCreatedLE = 0;
int nbROI = 0;
int nbRoiNotAnalysed = 0;
int roicounter = 0;

unsigned char * ptr;

// Loop pixel's subdivision.
for(int i = 0; i < subdivision.rows; i++) {

    ptr = subdivision.ptr<unsigned char>(i);

    for(int j = 0; j < subdivision.cols; j++) {

        // Pixel is white.
        if((int)ptr[j] > 0) {

            // Check if we are not out of frame range when a ROI is defined at the current pixel location.
            if((subdivisionPos.y + i - mRoiSize[1]/2 > 0) &&
               (subdivisionPos.y + i + mRoiSize[1]/2 < absDiffBinaryMap.rows) &&
               (subdivisionPos.x + j - mRoiSize[0]/2 > 0) &&
               (subdivisionPos.x + j + mRoiSize[0]/2 < absDiffBinaryMap.cols)) {

                msg = msg
                    + "Analyse ROI ("
                    +  Conversion::intToString(subdivisionPos.x + j) + ";" + Conversion::intToString(subdivisionPos.y + i) + ")\n";

                nbROI++;
                roicounter++;
                // Get colors in eventMap at the current ROI location.
                vector<cv::Scalar> listColorInRoi = getColorInEventMap(eventMap, cv::Point(subdivisionPos.x + j, subdivisionPos.y + i));

                if(listColorInRoi.size() == 0)  situation = 0;  // black color = create a new local event
                if(listColorInRoi.size() == 1)  situation = 1;  // one color = add the current roi to an existing local event
                if(listColorInRoi.size() >  1)  situation = 2;  // several colors = make a decision

                switch(situation) {

                    case 0 :

                        {

                            if(listLE.size() < maxNbLE) {

                                msg = msg
                                    + "->CREATE New Local EVENT\n"
                                    + "  - Initial position : ("
                                    +  Conversion::intToString(subdivisionPos.x + j) + ";" + Conversion::intToString(subdivisionPos.y + i) + ")\n"
                                    + "  - Color : (" + Conversion::intToString(mListColors.at(listLE.size())[0]) + ";"
                                    + Conversion::intToString(mListColors.at(listLE.size())[1]) + ";"
                                    + Conversion::intToString(mListColors.at(listLE.size())[2]) + ")\n";

                                // Create new localEvent object.
                                LocalEvent newLocalEvent(   mListColors.at(listLE.size()),
                                    cv::Point(subdivisionPos.x + j, subdivisionPos.y + i),
                                                            absDiffBinaryMap.rows,
                                                            absDiffBinaryMap.cols,
                                                            mRoiSize);

                                // Extract white pixels in ROI.
                                vector<cv::Point> whitePixAbsDiff,whitePixPosDiff, whitePixNegDiff;
                                cv::Mat roiAbsDiff, roiPosDiff, roiNegDiff;

                                absDiffBinaryMap(cv::Rect(subdivisionPos.x + j - mRoiSize[0]/2, subdivisionPos.y + i - mRoiSize[1]/2, mRoiSize[0], mRoiSize[1])).copyTo(roiAbsDiff);
                                posDiff(cv::Rect(subdivisionPos.x + j - mRoiSize[0]/2, subdivisionPos.y + i - mRoiSize[1]/2, mRoiSize[0], mRoiSize[1])).copyTo(roiPosDiff);
                                negDiff(cv::Rect(subdivisionPos.x + j - mRoiSize[0]/2, subdivisionPos.y + i - mRoiSize[1]/2, mRoiSize[0], mRoiSize[1])).copyTo(roiNegDiff);

                                if(roiPosDiff.type() == CV_16UC1 && roiNegDiff.type() == CV_16UC1) {

                                    unsigned char * ptrRoiAbsDiff;
                                    unsigned short * ptrRoiPosDiff;
                                    unsigned short * ptrRoiNegDiff;

                                    for(int a = 0; a < roiAbsDiff.rows; a++) {

                                        ptrRoiAbsDiff = roiAbsDiff.ptr<unsigned char>(a);
                                        ptrRoiPosDiff = roiPosDiff.ptr<unsigned short>(a);
                                        ptrRoiNegDiff = roiNegDiff.ptr<unsigned short>(a);

                                        for(int b = 0; b < roiAbsDiff.cols; b++){

                                            if(ptrRoiAbsDiff[b] > 0) whitePixAbsDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));
                                            if(ptrRoiPosDiff[b] > posDiffThreshold) whitePixPosDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));
                                            if(ptrRoiNegDiff[b] > negDiffThreshold) whitePixNegDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));

                                        }
                                    }

                                }else if(roiPosDiff.type() == CV_8UC1 && roiNegDiff.type() == CV_8UC1) {

                                    unsigned char * ptrRoiAbsDiff;
                                    unsigned char * ptrRoiPosDiff;
                                    unsigned char * ptrRoiNegDiff;

                                    for(int a = 0; a < roiAbsDiff.rows; a++) {

                                        ptrRoiAbsDiff = roiAbsDiff.ptr<unsigned char>(a);
                                        ptrRoiPosDiff = roiPosDiff.ptr<unsigned char>(a);
                                        ptrRoiNegDiff = roiNegDiff.ptr<unsigned char>(a);

                                        for(int b = 0; b < roiAbsDiff.cols; b++){

                                            if(ptrRoiAbsDiff[b] > 0) whitePixAbsDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));
                                            if(ptrRoiPosDiff[b] > posDiffThreshold) whitePixPosDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));
                                            if(ptrRoiNegDiff[b] > negDiffThreshold) whitePixNegDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));

                                        }
                                    }
                                }

                                msg = msg + "Number white pix in abs diff : " + Conversion::intToString(whitePixAbsDiff.size()) + "\n";
                                msg = msg + "Number white pix in pos diff : " + Conversion::intToString(whitePixPosDiff.size()) + "\n";
                                msg = msg + "Number white pix in neg diff : " + Conversion::intToString(whitePixNegDiff.size()) + "\n";

                                newLocalEvent.addAbs(whitePixAbsDiff);
                                newLocalEvent.addPos(whitePixPosDiff);
                                newLocalEvent.addNeg(whitePixNegDiff);

                                // Update center of mass.
                                newLocalEvent.computeMassCenter();
                                msg = msg
                                    + "  - Center of mass abs pixels : ("
                                    +  Conversion::intToString(newLocalEvent.getMassCenter().x) + ";" + Conversion::intToString(newLocalEvent.getMassCenter().y) + ")\n";

                                // Save the frame number where the local event has been created.
                                newLocalEvent.setNumFrame(numFrame);
                                // Save acquisition date of the frame.
                                newLocalEvent.mFrameAcqDate = cFrameDate;
                                // Add LE in the list of localEvent.
                                listLE.push_back(newLocalEvent);
                                // Update eventMap with the color of the new localEvent.
                                cv::Mat roi(mRoiSize[1], mRoiSize[0], CV_8UC3, mListColors.at(listLE.size()-1));
                                roi.copyTo(eventMap(cv::Rect(subdivisionPos.x + j-mRoiSize[0]/2, subdivisionPos.y + i-mRoiSize[1]/2, mRoiSize[0], mRoiSize[1])));
                                // Color roi in black in the current region.
                                colorRoiInBlack(cv::Point(j,i), mRoiSize[1], mRoiSize[0], subdivision);

                                colorRoiInBlack(cv::Point(subdivisionPos.x +j,subdivisionPos.y +i), mRoiSize[1], mRoiSize[0], absDiffBinaryMap);
                                colorRoiInBlack(cv::Point(subdivisionPos.x +j,subdivisionPos.y +i), mRoiSize[1], mRoiSize[0], posDiff);
                                colorRoiInBlack(cv::Point(subdivisionPos.x +j,subdivisionPos.y +i), mRoiSize[1], mRoiSize[0], negDiff);

                                nbCreatedLE++;

                            }else {

                                nbNoCreatedLE++;

                            }
                        }

                        break;

                    case 1 :

                        {

                            vector<LocalEvent>::iterator it;
                            int index = 0;
                            for(it=listLE.begin(); it!=listLE.end(); ++it){

                                // Try to find a local event which has the same color.
                                if((*it).getColor() == listColorInRoi.at(0)){

                                    msg = msg
                                        + "->Attach ROI ("
                                        +  Conversion::intToString(subdivisionPos.x + j) + ";" + Conversion::intToString(subdivisionPos.y + i) + ") with LE " + Conversion::intToString(index) + "\n";

                                    // Extract white pixels in ROI.
                                    vector<cv::Point> whitePixAbsDiff,whitePixPosDiff, whitePixNegDiff;
                                    cv::Mat roiAbsDiff, roiPosDiff, roiNegDiff;

                                    absDiffBinaryMap(cv::Rect(subdivisionPos.x + j - mRoiSize[0]/2, subdivisionPos.y + i - mRoiSize[1]/2, mRoiSize[0], mRoiSize[1])).copyTo(roiAbsDiff);
                                    posDiff(cv::Rect(subdivisionPos.x + j - mRoiSize[0]/2, subdivisionPos.y + i - mRoiSize[1]/2, mRoiSize[0], mRoiSize[1])).copyTo(roiPosDiff);
                                    negDiff(cv::Rect(subdivisionPos.x + j - mRoiSize[0]/2, subdivisionPos.y + i - mRoiSize[1]/2, mRoiSize[0], mRoiSize[1])).copyTo(roiNegDiff);

                                    if(roiPosDiff.type() == CV_16UC1 && roiNegDiff.type() == CV_16UC1) {

                                        unsigned char * ptrRoiAbsDiff;
                                        unsigned short * ptrRoiPosDiff;
                                        unsigned short * ptrRoiNegDiff;

                                        for(int a = 0; a < roiAbsDiff.rows; a++) {

                                            ptrRoiAbsDiff = roiAbsDiff.ptr<unsigned char>(a);
                                            ptrRoiPosDiff = roiPosDiff.ptr<unsigned short>(a);
                                            ptrRoiNegDiff = roiNegDiff.ptr<unsigned short>(a);

                                            for(int b = 0; b < roiAbsDiff.cols; b++){

                                                if(ptrRoiAbsDiff[b] > 0) whitePixAbsDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));
                                                if(ptrRoiPosDiff[b] > posDiffThreshold) whitePixPosDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));
                                                if(ptrRoiNegDiff[b] > negDiffThreshold) whitePixNegDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));

                                            }
                                        }

                                    }else if(roiPosDiff.type() == CV_8UC1 && roiNegDiff.type() == CV_8UC1) {

                                        unsigned char * ptrRoiAbsDiff;
                                        unsigned char * ptrRoiPosDiff;
                                        unsigned char * ptrRoiNegDiff;

                                        for(int a = 0; a < roiAbsDiff.rows; a++) {

                                            ptrRoiAbsDiff = roiAbsDiff.ptr<unsigned char>(a);
                                            ptrRoiPosDiff = roiPosDiff.ptr<unsigned char>(a);
                                            ptrRoiNegDiff = roiNegDiff.ptr<unsigned char>(a);

                                            for(int b = 0; b < roiAbsDiff.cols; b++){

                                                if(ptrRoiAbsDiff[b] > 0) whitePixAbsDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));
                                                if(ptrRoiPosDiff[b] > posDiffThreshold) whitePixPosDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));
                                                if(ptrRoiNegDiff[b] > negDiffThreshold) whitePixNegDiff.push_back(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2 + b, subdivisionPos.y + i - mRoiSize[1]/2 + a));

                                            }
                                        }
                                    }

                                    msg = msg + "Number white pix in abs diff : " + Conversion::intToString(whitePixAbsDiff.size()) + "\n";
                                    msg = msg + "Number white pix in pos diff : " + Conversion::intToString(whitePixPosDiff.size()) + "\n";
                                    msg = msg + "Number white pix in neg diff : " + Conversion::intToString(whitePixNegDiff.size()) + "\n";

                                    (*it).addAbs(whitePixAbsDiff);
                                    (*it).addPos(whitePixPosDiff);
                                    (*it).addNeg(whitePixNegDiff);

                                    // Add the current roi.
                                    (*it).mLeRoiList.push_back(cv::Point(subdivisionPos.x + j, subdivisionPos.y + i));
                                    // Set local event 's map
                                    (*it).setMap(cv::Point(subdivisionPos.x + j - mRoiSize[0]/2, subdivisionPos.y + i - mRoiSize[1]/2), mRoiSize[1], mRoiSize[0]);
                                    // Update center of mass
                                    (*it).computeMassCenter();
                                    msg = msg
                                    + "  - Update Center of mass abs pixels of LE " +  Conversion::intToString(index) + " : ("
                                    +  Conversion::intToString((*it).getMassCenter().x) + ";" + Conversion::intToString((*it).getMassCenter().y) + ")\n";

                                    // Update eventMap with the color of the new localEvent
                                    cv::Mat roi(mRoiSize[1], mRoiSize[0], CV_8UC3, listColorInRoi.at(0));
                                    roi.copyTo(eventMap(cv::Rect(subdivisionPos.x + j-mRoiSize[0]/2, subdivisionPos.y + i-mRoiSize[1]/2,mRoiSize[0],mRoiSize[1])));
                                    // Color roi in black in thresholded frame.
                                    cv::Mat roiBlack(mRoiSize[1],mRoiSize[0],CV_8UC1, cv::Scalar(0));
                                    roiBlack.copyTo(absDiffBinaryMap(cv::Rect(subdivisionPos.x + j-mRoiSize[0]/2, subdivisionPos.y + i-mRoiSize[1]/2,mRoiSize[0],mRoiSize[1])));
                                    // Color roi in black in the current region.
                                    colorRoiInBlack(cv::Point(j,i), mRoiSize[1], mRoiSize[0], subdivision);

                                    colorRoiInBlack(cv::Point(subdivisionPos.x +j,subdivisionPos.y +i), mRoiSize[1], mRoiSize[0], absDiffBinaryMap);
                                    colorRoiInBlack(cv::Point(subdivisionPos.x +j,subdivisionPos.y +i), mRoiSize[1], mRoiSize[0], posDiff);
                                    colorRoiInBlack(cv::Point(subdivisionPos.x +j,subdivisionPos.y +i), mRoiSize[1], mRoiSize[0], negDiff);

                                    nbRoiAttachedToLE++;

                                    break;

                                }

                                index++;
                            }
                        }

                        break;

                    case 2 :

                        {
                            nbRoiNotAnalysed++;

                           /* vector<LocalEvent>::iterator it;
                            vector<LocalEvent>::iterator itLEbase;
                            it = listLE.begin();

                            vector<Scalar>::iterator it2;
                            it2 = listColorInRoi.begin();

                            bool LE = false;
                            bool colorFound = false;

                            while (it != listLE.end()){

                                // Check if the current LE have a color.
                                while (it2 != listColorInRoi.end()){

                                    if((*it).getColor() == (*it2)){

                                       colorFound = true;
                                       it2 = listColorInRoi.erase(it2);
                                       break;

                                    }

                                    ++it2;
                                }

                                if(colorFound){

                                    if(!LE){

                                        itLEbase = it;
                                        LE = true;

                                        (*it).LE_Roi.push_back(Point(areaPosition.x + j, areaPosition.y + i));

                                        Mat tempMat = (*it).getMap();
                                        Mat roiTemp(roiSize[1],roiSize[0],CV_8UC1,Scalar(255));
                                        roiTemp.copyTo(tempMat(Rect(areaPosition.x + j - roiSize[0]/2, areaPosition.y + i - roiSize[1]/2, roiSize[0], roiSize[1])));
                                        (*it).setMap(tempMat);

                                        // Update center of mass
                                        (*it).computeMassCenterWithRoi();

                                        // Update eventMap with the color of the new localEvent group
                                        Mat roi(roiSize[1], roiSize[0], CV_8UC3, listColorInRoi.at(0));
                                        roi.copyTo(eventMap(Rect(areaPosition.x + j-roiSize[0]/2, areaPosition.y + i-roiSize[1]/2,roiSize[0],roiSize[1])));

                                        colorInBlack(j, i, areaPosX, areaPosY, areaPosition, area, frame);

                                    }else{

                                        // Merge LE data

                                        Mat temp = (*it).getMap();
                                        Mat temp2 = (*itLEbase).getMap();
                                        Mat temp3 = temp + temp2;
                                        (*itLEbase).setMap(temp3);

                                        (*itLEbase).LE_Roi.insert((*itLEbase).LE_Roi.end(), (*it).LE_Roi.begin(), (*it).LE_Roi.end());

                                        it = listLE.erase(it);

                                    }

                                    colorFound = false;

                                }else{

                                    ++it;

                                }
                            }*/

                        }

                        break;
                }
            }
        }
    }
}

msg = msg
    + "--> RESUME REGION ANALYSE : \n"
    + "Number of analysed ROI : "
    + Conversion::intToString(nbROI) + "\n"
    + "Number of not analysed ROI : "
    + Conversion::intToString(nbRoiNotAnalysed) + "\n"
    + "Number of new LE : "
    + Conversion::intToString(nbCreatedLE) + "\n"
    + "Number of updated LE :"
    + Conversion::intToString(nbRoiAttachedToLE) + "\n";

}


/*

// Create debug video.
if(dtp.DET_DEBUG_VIDEO)
    mVideoDebug = VideoWriter(mDebugCurrentPath + "debug-video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(static_cast<int>(1280), static_cast<int>(960)), true);

if(mdtp.DET_DEBUG_VIDEO){

    // Create GE memory image
    Mat GEMAP = Mat(currImg.rows, currImg.cols, CV_8UC3, Scalar(0,0,0));
    for(itGE = mListGlobalEvents.begin(); itGE!= mListGlobalEvents.end(); ++itGE){

        GEMAP = GEMAP + (*itGE).getGeMapColor();

    }

    if(mdtp.DET_DEBUG) SaveImg::saveBMP(GEMAP, mDebugCurrentPath + "/GEMAP/GEMAP_"+Conversion::intToString(c.mFrameNumber));

    Mat VIDEO               = Mat(960,1280, CV_8UC3,Scalar(255,255,255));
    Mat VIDEO_frame         = Mat(470,630, CV_8UC3,Scalar(0,0,0));
    Mat VIDEO_diffFrame     = Mat(470,630, CV_8UC3,Scalar(0,0,0));
    Mat VIDEO_threshFrame   = Mat(470,630, CV_8UC3,Scalar(0,0,0));
    Mat VIDEO_eventFrame    = Mat(470,630, CV_8UC3,Scalar(0,0,0));
    Mat VIDEO_geFrame       = Mat(470,630, CV_8UC3,Scalar(0,0,0));

    cvtColor(currImg, currImg, CV_GRAY2BGR);
    resize(currImg, VIDEO_frame, Size(630,470), 0, 0, INTER_LINEAR );
    cvtColor(absDiffBinaryMap, absDiffBinaryMap, CV_GRAY2BGR);
    resize(absDiffBinaryMap, VIDEO_threshFrame, Size(630,470), 0, 0, INTER_LINEAR );
    resize(eventMap, VIDEO_eventFrame, Size(630,470), 0, 0, INTER_LINEAR );
    resize(GEMAP, VIDEO_geFrame, Size(630,470), 0, 0, INTER_LINEAR );

    copyMakeBorder(VIDEO_frame, VIDEO_frame, 5, 5, 5, 5, BORDER_CONSTANT, Scalar(255,255,255) );
    copyMakeBorder(VIDEO_threshFrame, VIDEO_threshFrame, 5, 5, 5, 5, BORDER_CONSTANT, Scalar(255,255,255) );
    copyMakeBorder(VIDEO_eventFrame, VIDEO_eventFrame, 5, 5, 5, 5, BORDER_CONSTANT, Scalar(255,255,255) );
    copyMakeBorder(VIDEO_geFrame, VIDEO_geFrame, 5, 5, 5, 5, BORDER_CONSTANT, Scalar(255,255,255) );

    putText(VIDEO_frame, "Original", cvPoint(300,450),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
    putText(VIDEO_threshFrame, "Filtering", cvPoint(300,450),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
    putText(VIDEO_eventFrame, "Local Event Map", cvPoint(300,450),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
    putText(VIDEO_geFrame, "Global Event Map", cvPoint(300,450),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);

    VIDEO_frame.copyTo(VIDEO(Rect(0, 0, 640, 480)));
    VIDEO_threshFrame.copyTo(VIDEO(Rect(640, 0, 640, 480)));
    VIDEO_eventFrame.copyTo(VIDEO(Rect(0, 480, 640, 480)));
    VIDEO_geFrame.copyTo(VIDEO(Rect(640, 480, 640, 480)));

    string fn = Conversion::intToString(c.mFrameNumber);
    const char * fn_c;
    fn_c = fn.c_str();

    putText(VIDEO, fn_c, cvPoint(30,50),FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,255,0), 2, CV_AA);

    if(mVideoDebug.isOpened()){

        mVideoDebug << VIDEO;

    }

}

*/
