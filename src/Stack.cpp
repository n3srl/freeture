/*
                                Stack.cpp

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
* \file    Stack.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    19/06/2014
* \brief
*/
//header refactoring ok
#include "Stack.h"

#include <boost/date_time.hpp>

#include "Fits2D.h"

#include "Logger.h"

using namespace freeture;
using namespace std;

Stack::Stack(string fitsCompression, fitskeysParam fkp, stationParam stp):
mFitsCompressionMethod(fitsCompression),
curFrames(0), varExpTime(false),
sumExpTime(0.0), gainFirstFrame(0), expFirstFrame(0), fps(0), format(CamPixFmt::MONO8){
    mfkp = fkp;
    mstp = stp;
}

Stack::~Stack(){}

void Stack::addFrame(shared_ptr<Frame> i){

    try{

        if(TimeDate::getYYYYMMDD(i->mDate) != "00000000") {

            if(curFrames == 0){

                cv::Mat zero_mat = cv::Mat::zeros(i->Image->rows, i->Image->cols, CV_32FC1);
                    
                stack = make_shared<cv::Mat>(std::move(zero_mat));

                gainFirstFrame = i->mGain;
                expFirstFrame = i->mExposure;
                mDateFirstFrame = i->mDate;
                fps = i->mFps;
                format = i->mFormat;

            }

            if(expFirstFrame != i->mExposure)
                varExpTime = true;

            sumExpTime+=i->mExposure;

            cv::Mat curr = cv::Mat::zeros(i->Image->rows, i->Image->cols, CV_32FC1);

            i->Image->convertTo(curr, CV_32FC1);

            accumulate(curr, *stack.get());
            curFrames++;
            mDateLastFrame = i->mDate;

        }

    }catch(exception& e){
        LOG_DEBUG << e.what() << endl;
        LOG_ERROR << e.what() ;
    }

}

bool Stack::saveStack(string path, StackMeth stackMthd, bool stackReduction){
    double  debObsInSeconds = mDateFirstFrame.hours*3600 + mDateFirstFrame.minutes*60 + mDateFirstFrame.seconds;
    double  endObsInSeconds = mDateLastFrame.hours*3600 + mDateLastFrame.minutes*60 + mDateLastFrame.seconds;
    double  elapTime        = endObsInSeconds - debObsInSeconds;
    double  julianDate      = TimeDate::gregorianToJulian(mDateFirstFrame);
    double  julianCentury   = TimeDate::julianCentury(julianDate);
    double  sideralT        = TimeDate::localSideralTime_2(julianCentury, mDateFirstFrame.hours,  mDateFirstFrame.minutes, (int)mDateFirstFrame.seconds, mstp.SITELONG);

    LOG_INFO << "Start create fits2D to save the stack.";

    // Fits creation.
    Fits2D newFits(path);
    newFits.loadKeys(mfkp, mstp);
    LOG_INFO << "Fits path : " << path;
    // Creation date of the fits file : YYYY-MM-DDTHH:MM:SS
    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
    LOG_INFO << "Setting Fits DATE (creation date) key : " << to_iso_extended_string(time);
    newFits.kDATE = to_iso_extended_string(time);
    // Frame exposure time (sec.)
    LOG_INFO << "Setting fits ONTIME (Frame exposure time (sec.)) key : " << sumExpTime/1000000.0;
    newFits.kONTIME = sumExpTime/1000000.0;
    // Detector gain
    LOG_INFO << "Setting fits GAIN key : " << gainFirstFrame;
    newFits.kGAINDB = gainFirstFrame;
    // Acquisition date of the first frame 'YYYY-MM-JJTHH:MM:SS.SS'
    newFits.kDATEOBS = TimeDate::getIsoExtendedFormatDate(mDateFirstFrame);

    if(varExpTime)
        newFits.kEXPOSURE = 999999;
    else
        newFits.kEXPOSURE = expFirstFrame/1000000.0;

    // end obs. date - start obs. date (sec.)
    LOG_INFO << "Setting fits ELAPTIME (end obs. date - start obs. date (sec.)) key : " << elapTime;
    newFits.kELAPTIME = elapTime;
    // Sideral time
    LOG_INFO << "Setting fits CRVAL1 (sideraltime) key : " << sideralT;
    newFits.kCRVAL1 = sideralT;
    // Fps
    LOG_INFO << "Setting fits CD3_3 (fps) key : " << fps;
    newFits.kCD3_3 = (double)fps;
    // Projection and reference system
    LOG_INFO << "Setting fits DATEOBS key : RA---ARC";
    newFits.kCTYPE1 = "RA---ARC";
    LOG_INFO << "Setting fits DATEOBS key : DEC--ARC";
    newFits.kCTYPE2 = "DEC--ARC";
    // Equinox
    LOG_INFO << "Setting fits DATEOBS key : 2000.0";
    newFits.kEQUINOX = 2000.0;

    switch(stackMthd) {

        case MEAN :

            {

                LOG_INFO << "MEAN STACK MODE";

                // 'SINGLE' 'SUM' 'AVERAGE' ('MEDIAN')
                newFits.kOBSMODE = "AVERAGE";

                cv::Mat new_stack = *stack.get() / curFrames;
                stack = make_shared<cv::Mat>(std::move(new_stack));


                switch(format) {

                    case CamPixFmt::MONO12 :

                        {
                        LOG_INFO << "Mono12 format";

                            shared_ptr<cv::Mat> newMat = make_shared<cv::Mat>(stack->rows,stack->cols, CV_16SC1, cv::Scalar(0));

                            LOG_INFO << "Setting fits BZERO key : 32768";
                            newFits.kBZERO = 32768;
                            LOG_INFO << "Setting fits BSCALE key : 1";
                            newFits.kBSCALE = 1;
                            LOG_INFO << "Setting fits SATURATE key : 4095";
                            newFits.kSATURATE = 4095;


                            float *ptr = NULL;
                            short *ptr2 = NULL;

                            for(int i = 0; i < stack->rows; i++){

                                ptr = stack->ptr<float>(i);
                                ptr2 = newMat->ptr<short>(i);

                                for(int j = 0; j < stack->cols; j++){

                                    if(ptr[j] - 32768 > 32767){

                                        ptr2[j] = 32767;

                                    }else{

                                        ptr2[j] = ptr[j] - 32768;
                                    }
                                }
                            }

                            LOG_INFO << "Writing FITS signed short image.";
                            return newFits.writeFits(newMat, S16, "", mFitsCompressionMethod);

                        }

                        break;

                    default :

                        {
                        LOG_INFO << "Mono8 format";

                        LOG_INFO << "Setting fits SATURATE key : 255";
                            newFits.kSATURATE = 255;

                            shared_ptr<cv::Mat> newMat = make_shared<cv::Mat>(stack->rows,stack->cols, CV_8UC1, cv::Scalar(0));

                            float * ptr;
                            unsigned char * ptr2;

                            for(int i = 0; i < stack->rows; i++){

                                ptr = stack->ptr<float>(i);
                                ptr2 = newMat->ptr<unsigned char>(i);

                                for(int j = 0; j < stack->cols; j++){

                                    ptr2[j] = (unsigned char)ptr[j];

                                }
                            }

                            // Create FITS image with BITPIX = BYTE_IMG (8-bits unsigned integers), pixel with TBYTE (8-bit unsigned byte)
                            LOG_INFO << "Writing FITS image with BITPIX = BYTE_IMG (8-bits unsigned integers), pixel with TBYTE (8-bit unsigned byte)";
                            return newFits.writeFits(newMat, UC8, "" , mFitsCompressionMethod);

                        }

                }

            }

            break;

        case SUM :

            {
            LOG_INFO << "SUM STACK MODE";

                // 'SINGLE' 'SUM' 'AVERAGE' ('MEDIAN')
                newFits.kOBSMODE = "SUM";


                if(format == CamPixFmt::MONO12){
                    LOG_INFO << "Setting fits SATURATE key : 4095 * curFrames";
                    newFits.kSATURATE = 4095 * curFrames;
                }
                else {
                    LOG_INFO << "Setting fits SATURATE key : 255 * curFrames";
                    newFits.kSATURATE = 255 * curFrames;
                }

                if(stackReduction){

                    LOG_INFO << "stackReduction option enabled";

                    shared_ptr<cv::Mat> newMat ;

                    float bzero  = 0.0;
                    float bscale = 1.0;

                    LOG_INFO << "Call reduction function.";
                    reductionByFactorDivision(bzero, bscale)->copyTo(*(newMat.get()));

                    LOG_INFO << "Setting fits BZERO key : " << bzero;
                    newFits.kBZERO = bzero;
                    LOG_INFO << "Setting fits BSCALE key : " << bscale;
                    newFits.kBSCALE = bscale;

                    switch(format){

                        case CamPixFmt::MONO12 :

                            {
                            LOG_INFO << "Writting Fits signed short.";
                                return newFits.writeFits(newMat, S16, "", mFitsCompressionMethod);

                            }

                            break;

                        default :

                            {
                            LOG_INFO << "Writting Fits unsigned char.";
                                return newFits.writeFits(newMat, UC8, "", mFitsCompressionMethod);

                            }

                    }

                }else{

                    // Save fits in 32 bits.
                    LOG_INFO << "Writting Fits 32 bits.";
                    return newFits.writeFits(stack, F32, ""  );

                }

            }

            break;

        default: 
            return false;
    }

    return true;
}

shared_ptr<cv::Mat> Stack::reductionByFactorDivision(float &bzero, float &bscale){

    shared_ptr<cv::Mat> newMat;

    switch(format){

    case CamPixFmt::MONO12 :

            {

                newMat = make_shared<cv::Mat>(stack->rows,stack->cols, CV_16SC1, cv::Scalar(0));
                float factor = (4095.0f * curFrames)/4095.0f;

                bscale = factor;
                bzero  = 32768 * factor;

                float * ptr;
                short * ptr2;

                for(int i = 0; i < stack->rows; i++){

                    ptr = stack->ptr<float>(i);
                    ptr2 = newMat->ptr<short>(i);

                    for(int j = 0; j < stack->cols; j++){

                        if(cvRound(ptr[j] / factor) - 32768 > 32767){

                            ptr2[j] = 32767;

                        }else{

                            ptr2[j] = cvRound(ptr[j] / factor) - 32768;
                        }
                    }
                }
            }

            break;

        default :

            {

                newMat = make_shared<cv::Mat>(stack->rows,stack->cols, CV_8UC1, cv::Scalar(0));
                float factor = curFrames;
                bscale = factor;
                bzero  = 0;

                float * ptr;
                unsigned char * ptr2;

                for(int i = 0; i < stack->rows; i++){

                    ptr = stack->ptr<float>(i);
                    ptr2 = newMat->ptr<unsigned char>(i);

                    for(int j = 0; j < stack->cols; j++){

                        ptr2[j] = cvRound(ptr[j] / factor) ;

                    }
                }
            }

    }

    return newMat;

}
