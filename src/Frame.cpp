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

#include "Logger.h"

#include "ECamPixFmt.h"

using namespace freeture;

 Frame::Frame(cv::Mat capImg, int gain, double exposure_time, std::string acquisitionDate):
     mDataBuffer(nullptr),
     mSize(0),
     mDate(TimeDate::splitIsoExtendedDate(acquisitionDate)),
     mExposure(exposure_time),
     mGain(gain),
     mFormat(CamPixFmt::MONO8),
     mFileName("noFileName"),
     mFrameNumber(0),
     mFrameRemaining(0),
     mSaturatedValue(255),
     mFps(0),
     mStartX(0),
     mStartY(0),
     mWidth(capImg.cols),
     mHeight(capImg.rows),
     Image(std::make_shared<cv::Mat>(capImg.clone())) // Deep copy of capImg
 {
   
 }

Frame::Frame():
    mDataBuffer(nullptr),
    mSize(0),
    mDate(),
    mExposure(0.0),
    mGain(0),
    mFormat(CamPixFmt::MONO8),
    mFileName("noFileName"),
    mFrameNumber(0),
    mFrameRemaining(0),
    mSaturatedValue(255),
    mFps(0),
    mStartX(0),
    mStartY(0),
    mWidth(0),
    mHeight(0),
    Image()
{
}

Frame::Frame(const Frame& other) 
    :
        mSize(other.mSize),
        mDate(other.mDate),
        mExposure(other.mExposure),
        mGain(other.mGain),
        mFormat(other.mFormat),
        mFileName(other.mFileName),
        mFrameNumber(other.mFrameNumber),
        mFrameRemaining(other.mFrameRemaining),
        mSaturatedValue(other.mSaturatedValue),
        mFps(other.mFps),
        mStartX(other.mStartX),
        mStartY(other.mStartY),
        mWidth(other.mWidth),
        mHeight(other.mHeight)
{
    if (other.Image) {
        Image = std::make_shared<cv::Mat>(other.Image->clone());
    }
    else {
        Image = std::make_shared<cv::Mat>();
    }

    if (other.mDataBuffer) {
        mDataBuffer = new uint8_t[other.mSize];
        std::copy(other.mDataBuffer, other.mDataBuffer + other.mSize, mDataBuffer);
    }
    else {
        mDataBuffer = nullptr;
    }
}

Frame::Frame(Frame&& other) noexcept
    :
    Image(std::move(other.Image)), // Transfer ownership of the shared_ptr
    mDataBuffer(other.mDataBuffer), // Transfer the raw pointer
    mSize(other.mSize), // Copy the size
    // Copy or move other members as needed
    mDate(std::move(other.mDate)),
    mExposure(other.mExposure),
    mGain(other.mGain),
    mFormat(other.mFormat),
    mFileName(std::move(other.mFileName)),
    mFrameNumber(other.mFrameNumber),
    mFrameRemaining(other.mFrameRemaining),
    mSaturatedValue(other.mSaturatedValue),
    mFps(other.mFps),
    mStartX(other.mStartX),
    mStartY(other.mStartY),
    mWidth(other.mWidth),
    mHeight(other.mHeight)
{
    other.mDataBuffer = nullptr; // Ensure the source object doesn't delete the data
    other.mSize = 0; // Reset size in the source object
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

Frame& Frame::operator=(const Frame& other)
{
    if (this != &other) // protect against invalid self-assignment
    { 
        // Deep copy of Image
        if (other.Image) {
            Image = std::make_shared<cv::Mat>(other.Image->clone());
        }
        else {
            Image = std::make_shared<cv::Mat>();
        }

        // Deep copy of mDataBuffer
        if (mDataBuffer) {
            delete[] mDataBuffer;
            mDataBuffer = nullptr;
            mSize = 0;
        }

        if (other.mDataBuffer && other.mSize > 0) {
            mDataBuffer = new uint8_t[other.mSize];
            std::copy(other.mDataBuffer, other.mDataBuffer + other.mSize, mDataBuffer);
            mSize = other.mSize;
        }

        // Copying simple fields
        mDate = other.mDate;
        mExposure = other.mExposure;
        mGain = other.mGain;
        mFormat = other.mFormat;
        mFileName = other.mFileName;
        mFrameNumber = other.mFrameNumber;
        mFrameRemaining = other.mFrameRemaining;
        mSaturatedValue = other.mSaturatedValue;
        mFps = other.mFps;
        mStartX = other.mStartX;
        mStartY = other.mStartY;
        mWidth = other.mWidth;
        mHeight = other.mHeight;
    }
    return *this;
}

Frame& Frame::operator=(Frame&& other) noexcept {
    if (this != &other) { // Prevent self-assignment
        Image = std::move(other.Image); // Transfer ownership of the shared_ptr
        delete[] mDataBuffer; // Free existing buffer
        mDataBuffer = other.mDataBuffer; // Transfer the raw pointer
        mSize = other.mSize; // Copy the size
        // Move or copy other members as needed
        mDate = std::move(other.mDate);
        mExposure = other.mExposure;
        mGain = other.mGain;
        mFormat = other.mFormat;
        mFileName = std::move(other.mFileName);
        mFrameNumber = other.mFrameNumber;
        mFrameRemaining = other.mFrameRemaining;
        mSaturatedValue = other.mSaturatedValue;
        mFps = other.mFps;
        mStartX = other.mStartX;
        mStartY = other.mStartY;
        mWidth = other.mWidth;
        mHeight = other.mHeight;

        // Reset the moved-from object to a valid state
        other.mDataBuffer = nullptr;
        other.mSize = 0;
    }
    return *this;
}
