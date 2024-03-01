#pragma once
/*
                                Fits3D.h

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
* \file    Fits3D.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    12/03/2018
* \brief   Write fits3D file.
*/
#include "Commons.h"

#include <opencv2/opencv.hpp>

#include <string>

#include "Fits.h"
#include "ECamPixFmt.h"

#include "fitsio.h"

namespace freeture
{
    class Fits3D : public Fits {

    private:

        fitsfile* fptr;
        const char* mFileName;
        int             status;
        long            naxis;
        long            naxes[3];
        int             size3d;
        long            fpixel[3];
        int             imgSize;
        CamPixFmt       imgDepth;
        int             n;                  // Index for the number of images
        unsigned char* array3D_MONO_8;
        unsigned short* array3D_MONO_12;

    public:

        /**
        * Constructor.
        * @param depth Image format.
        * @param imgHeight Height of images.
        * @param imgWidth Width of images.
        * @param numberOfImages Number of images to add in the fits cube.
        * @param fileName Name od the fits cube.
        *
        */
        Fits3D(CamPixFmt depth, int imgHeight, int imgWidth, int numberOfImages, std::string fileName);

        /**
        * Constructor.
        *
        */
        Fits3D() :
            fptr(NULL), mFileName("noFileName"), status(0), naxis(3), size3d(0), imgSize(0),
            imgDepth(CamPixFmt::MONO8), n(0), array3D_MONO_12(NULL), array3D_MONO_8(NULL) {

        };

        /**
        * Destructor.
        *
        */
        ~Fits3D() {};

        void addImageToFits3D(cv::Mat frame);

        /**
        * Create and write fits 3D.
        * @param status Error cfitsio status.
        *
        */
        bool writeFits3D();

    private:

        /**
        * Helper function to get cfitsio error.
        * @param status Error cfitsio status.
        * @param errorMsg Additional information about where the error occured.
        *
        */
        void printerror(int status, std::string errorMsg);

        /**
        * Helper function to get cfitsio error.
        * @param status Error cfitsio status.
        *
        */
        void printerror(int status);

        /**
        * Write keywords in fits.
        * @param fptr Pointer on the fits.
        * @return Success to write keywords.
        *
        */
        bool writeKeywords();


    };
}
