/*
                            CameraFrames.cpp

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
* \file    CameraFrames.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    02/09/2014
* \brief   Fits frames in input of acquisition thread.
*/
//header refactoring ok
#include "CameraFrames.h"

#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

#include "CameraScanner.h"
#include "Conversion.h"
#include "Fits2D.h"
#include "Logger.h"

namespace fs = boost::filesystem;
using namespace std;
using namespace freeture;

CameraFrames::CameraFrames( CameraDescription camera_descriptor, cameraParam settings,  std::vector<std::string> locationList, int numPos, bool verbose) :
    Camera(camera_descriptor, settings),
    mNumFramePos(numPos),
    mReadDataStatus(false),
    mCurrDirId(0),
    mFirstFrameNum(0),
    mLastFrameNum(0) {

    if(locationList.size()>0)
        mFramesDir = locationList;
    else
        throw "No frames directory in input.";

    m_ExposureAvailable = false;
    m_GainAvailable = false;
    m_CameraDescriptor.DeviceType= SINGLE_FITS_FRAME;
}

CameraFrames::~CameraFrames(void) {

}

bool CameraFrames::loadNextDataSet(string &location) {

    LOG_DEBUG << mCurrDirId << endl;

    location = mFramesDir.at(mCurrDirId);

    //if(mCurrDirId !=0 ) {

        mReadDataStatus = false;

        if(!searchMinMaxFramesNumber(mFramesDir.at(mCurrDirId)))
            return false;

    //}

    return true;

}

bool CameraFrames::grabInitialization() {

    return searchMinMaxFramesNumber(mFramesDir.at(mCurrDirId));

}

bool CameraFrames::getDataSetStatus() {

    mCurrDirId++;

    if(mCurrDirId >= mFramesDir.size()) return false;
    else return true;
}

bool CameraFrames::getCameraName() {
    LOG_DEBUG << "Fits frames data." << endl;
    return true;
}

bool CameraFrames::searchMinMaxFramesNumber(string location) {

    namespace fs = boost::filesystem;

    fs::path p(location);

    if(fs::exists(p)){

        LOG_DEBUG<< "Frame's directory exists : " << location;

        int firstFrame = -1, lastFrame = 0;
        string filename = "";

        // Search first and last frames numbers in the directory.
        for(fs::directory_iterator file(p);file!= fs::directory_iterator(); ++file) {

            fs::path curr(file->path());

            if(is_regular_file(curr)) {

                // Get file name.
                string fname = curr.filename().string();

                // Split file name according to the separator "_".
                vector<string> output;
                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                boost::char_separator<char> sep("_");
                tokenizer tokens(fname, sep);

                for (tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter) {
                    output.push_back(*tok_iter);
                }

                // Search frame number according to the number position known in the file name.

                int i = 0, number = 0;

                for(int j = 0; j < output.size(); j++) {

                    if(j == mNumFramePos && j != output.size() - 1) {

                        number = atoi(output.at(j).c_str());
                        break;
                    }

                    // If the frame number is at the end (before the file extension).
                    if(j == mNumFramePos && j == output.size() - 1) {

                        vector<string> output2;
                        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                        boost::char_separator<char> sep2(".");
                        tokenizer tokens2(output.back(), sep2);

                        for (tokenizer::iterator tok_iter = tokens2.begin();tok_iter != tokens2.end(); ++tok_iter) {
                            output2.push_back(*tok_iter);
                        }

                        number = atoi(output2.front().c_str());
                        break;
                    }

                    i++;
                }

                if(firstFrame == -1) {

                    firstFrame = number;

                }else if(number < firstFrame) {

                    firstFrame = number;

                }

                if(number > lastFrame) {

                    lastFrame = number;

                }
            }

        }

        LOG_DEBUG << "First frame number in frame's directory : " << firstFrame;

        mLastFrameNum = lastFrame;
        mFirstFrameNum = firstFrame;

        return true;

    }else{
        LOG_ERROR << "Frame's directory not found.";
        return false;

    }

}

bool CameraFrames::getStopStatus() {

    return mReadDataStatus;

}

bool CameraFrames::getFPS(double &value) {

    value = 0;
    return false;

}

bool CameraFrames::grabImage(Frame &img) {

    bool fileFound = false;

    string filename = "";

    fs::path p(mFramesDir.at(mCurrDirId));

    /// Search a frame in the directory.
    for(fs::directory_iterator file(p);file!= fs::directory_iterator(); ++file){

        fs::path curr(file->path());

        if(is_regular_file(curr)){

            list<string> ch;
            string fname = curr.filename().string();
            Conversion::stringTok(ch, fname.c_str(), "_");
            list<string>::const_iterator lit(ch.begin()), lend(ch.end());
            int i = 0;
            int number = 0;

            for(; lit != lend; ++lit){

                if(i == mNumFramePos && i != ch.size() - 1){

                    number = atoi((*lit).c_str()); break;
                }

                if(i == ch.size() - 1){

                    list<string> ch_;
                    Conversion::stringTok(ch_, (*lit).c_str(), ".");
                    number = atoi(ch_.front().c_str());
                    break;

                }

                i++;

            }

            if(number == mFirstFrameNum){

                mFirstFrameNum++;
                fileFound = true;

                LOG_DEBUG << "FILE:" << file->path().string() << endl;
                LOG_INFO <<  "FILE:" << file->path().string();

                filename = file->path().string() ;

                break;

            }
        }
    }


    if(mFirstFrameNum > mLastFrameNum || !fileFound){

        mReadDataStatus = true;
        LOG_INFO <<  "End read frames.";
        return false;

    }else{

        LOG_INFO <<  "Frame found.";

        Fits2D newFits(filename);
        int bitpix;

        if(!newFits.readIntKeyword("BITPIX", bitpix)){
            LOG_ERROR << " Fail to read fits keyword : BITPIX";

            return false;
        }

        /// Read the frame.

        cv::Mat resMat;
        CamPixFmt frameFormat = CamPixFmt::MONO8;

        switch(bitpix){

            case 8 :

                frameFormat = CamPixFmt::MONO8;
                newFits.readFits8UC(resMat);

                break;

            case 16 :

                frameFormat = CamPixFmt::MONO12;
                newFits.readFits16S(resMat);

                break;

        }

        boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
        Frame f;

        //Frame f = Frame( resMat, 0, 0, to_iso_extended_string(time));

        img = f;

        img.mFrameNumber = mFirstFrameNum -1 ;
        img.mFrameRemaining = mLastFrameNum - mFirstFrameNum-1;
        img.mFps = 1;
        img.mFormat = frameFormat;

        //waitKey(1000);


        return true;

    }

}

bool CameraFrames::initSDK()
{

    return true;
}

bool CameraFrames::initOnce()
{
    return true;
}

bool CameraFrames::init()
{
    return true;
}

void CameraFrames::fetchBounds(parameters&)
{

}

void CameraFrames::configure(parameters&)
{

}

bool CameraFrames::configurationCheck(parameters&)
{
    return true;
}

bool CameraFrames::createDevice()
{
    return true;
}

double CameraFrames::getMinExposureTime()
{
    return 0.0;
}


bool CameraFrames::destroyDevice()
{
    return false;
}