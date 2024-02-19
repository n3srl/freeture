/*
                                    Frame.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau 
*                       2018 Chiara Marmo
*                                       GEOPS-UPSUD
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
* \file    Frame.cpp
* \author  Yoan Audureau -- Chiara Marmo -- GEOPS-UPSUD
* \version 1.2
* \date    19/03/2018
* \brief   Frame grabbed from a camera or other input video source.
*/

#include "Frame.h"
#include "ECamPixFmt.h"
#include "Logger.h"

using namespace freeture;

// Frame::Frame(cv::Mat capImg, int g, double e, std::string acquisitionDate):
// mExposure(e), mGain(g), mFileName("noFileName"), mFrameRemaining(0),
// mFrameNumber(0), mFps(30), mFormat(CamPixFmt::MONO8), mSaturatedValue(255), mDataBuffer(nullptr)
// {
//     capImg.copyTo(*Image.get());
//     mDate = TimeDate::splitIsoExtendedDate(acquisitionDate);
//     mStartX = 0;
//     mStartY = 0;
//     mWidth = 0;
//     mHeight = 0;
// }

Frame::Frame():
mExposure(0), mGain(0), mFileName("noFileName"), mFrameRemaining(0),
mFrameNumber(0), mFps(30), mFormat(CamPixFmt::MONO8), mSaturatedValue(255), mDataBuffer(nullptr) {

   mWidth = 0;
   mHeight = 0;
   mStartX = 0;
   mStartY = 0;
}

Frame::~Frame()
{
    if (Image) {
        Image->release();
        Image->deallocate();
    }

    if (mDataBuffer)
        delete[] mDataBuffer;
}

void Frame::SetImage(const uint8_t* buffer, size_t size)
{
    if (mDataBuffer != nullptr)
        throw "deallocate first";

    if (buffer == nullptr)
        return;

    if (size<=0)
        return;

    size_t expected_buffer_size = mWidth * mHeight;

    switch (mFormat) {
    case CamPixFmt::MONO8: {

        break;
    }
    case CamPixFmt::MONO12:
    case CamPixFmt::MONO16: {
        expected_buffer_size *= 2;
        break;
    }
    };

    if (expected_buffer_size != size) 
    {
        if (LOG_SPAM_FRAME_STATUS)
            LOG_WARNING << "Frame::SetImage; Expected buffer size is different from frame size.";
    }

    
     mDataBuffer = new uint8_t[expected_buffer_size];
     memset(mDataBuffer,0xCD, expected_buffer_size);
     mSize = size;
     memcpy(mDataBuffer, buffer, size);
}

uint8_t* Frame::getData()
{
//     uint8_t* new_buffer = new uint8_t[mSize];
//     memset(new_buffer, 0xCD, mSize);
//     memcpy(mDataBuffer, new_buffer, mSize);
    return mDataBuffer;
}
