#pragma once

/*
                                Stack.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau -- GEOPS-UPSUD
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
* \file    Stack.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    19/06/2014
* \brief
*/
//header refactoring ok
#include "Commons.h"

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "TimeDate.h"
#include "ECamPixFmt.h"
#include "SParam.h"
#include "Frame.h"

namespace freeture
{
    class Stack {

    private:

      
        std::shared_ptr<cv::Mat>  stack;
        int             curFrames;
        int             gainFirstFrame;
        int             expFirstFrame;
        TimeDate::Date  mDateFirstFrame;
        TimeDate::Date  mDateLastFrame;
        int             fps;
        CamPixFmt       format;
        bool            varExpTime;
        double          sumExpTime;
        std::string          mFitsCompressionMethod;
        stationParam mstp;
        fitskeysParam mfkp;

    public:

        /**
        * Constructor.
        * Initializes a Stack instance with parameters related to FITS compression, FITS keys, 
        * and station parameters. This setup suggests that every stack is associated with specific 
        * imaging parameters and metadata that will likely be included in the FITS file header upon saving.
        */
        Stack(std::string fitsCompression, fitskeysParam fkp, stationParam stp);
        
        /// <summary>
        /// Copy constructor
        /// </summary>
        /// <param name="other"></param>
        Stack(const Stack& other);
        Stack& operator=(const Stack& other);

        /// <summary>
        /// Move constructor
        /// </summary>
        Stack(Stack&& other) noexcept;
        Stack& operator=(Stack&& other) noexcept;

        /**
        * Destructor.
        * Cleans up the resources. Since you're using std::shared_ptr for managing the cv::Mat object,
        * explicit cleanup in the destructor is not necessary for the cv::Mat object unless there are other resources
        * that need explicit release.
        */
        ~Stack(void);

        /**
        * Add a frame to the stack.
        *
        * Adds a frame to the stack. This method likely involves some operations to combine the incoming 
        * frame with the existing stack, such as averaging or summing pixel values. The exact implementation
        * depends on how you intend to "stack" these frames (e.g., for noise reduction, exposure stacking, etc.).
        * 
        * @param i Frame to add.
        */
        void addFrame(std::shared_ptr<Frame> i);

        /**
        * Get Date of the first frame of the stack.
        *
        * These methods provide access to specific metadata about the stack, such as the date of the
        * first frame added and the total number of frames. This information could be useful for analysis,
        * logging, or inclusion in the FITS header
        * 
        * @return Date.
        */
        TimeDate::Date getDateFirstFrame() { return mDateFirstFrame; };

        /**
        * Save stack.
        * 
        * Saves the stack to a file, presumably in the FITS format given the context.
        * This method would involve constructing the FITS header using the metadata from the
        * frames and the stack parameters, applying any final processing or reduction to the stack, 
        * and writing the result to a file.
        *
        * @param fitsHeader fits keywords.
        * @param path Location where to save the stack.
        * @param stackMthd Method to use to create stack.
        * @param stationName Name of the station.
        * @param stackReduction Enable stack reduction.
        * @return Success status.
        */
        bool saveStack(std::string path, StackMeth stackMthd, bool stackReduction);

        /**
        * Get number of frames in the stack.
        *
        * @return Number of frames.
        */
        int getNbFramesStacked() { return curFrames; };

        std::shared_ptr<cv::Mat> getStack() { return stack; };

        /**
        * Reduce stack in float 32 bits to 8 or 16 bits.
        *
        * @param bzero (Physical_value = image_value * bscale + bzero)
        * @param bscale
        * @return Reduced image.
        */
        std::shared_ptr<cv::Mat> reductionByFactorDivision(float& bzero, float& bscale);

    };
}
