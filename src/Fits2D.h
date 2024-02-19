#pragma once
/*
                                Fits2D.h

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
* \file    Fits2D.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    12/03/2018
* \brief   Write/Read fits2D file.
*/
#include "Commons.h"

#include <string>
#include <memory>

#include "fitsio.h"
#include "EImgBitDepth.h"
#include "Fits.h"

namespace cv 
{
    class Mat;
}

namespace freeture
{
    class Fits2D : public Fits {

    private:

        // Location where to save fits or path of a fits file to read.
        std::string mFitsPath;

    public:

        /**
        * Constructor.
        *
        */
        Fits2D(std::string path);

        /**
        * Destructor.
        *
        */
        ~Fits2D(void);

        /**
        * Create and write Fits image.
        * @param img Image to save in fits format.
        * @param imgType Format of the image (8 bits signed/unsigned char, 16 bits signed/unsigned char, 32 bits float).
        * @param fileName Optional parameter to specify a name of the fits file.
        * @return Success status to create and write the file.
        *
        */
        bool writeFits(std::shared_ptr<cv::Mat> img, ImgBitDepth imgType, std::string fileName, std::string compression = "");

        /**
        * Read a Fits file in 32 bits float format.
        * @param img Reference on the container which will contain the read fits.
        *
        */
        bool readFits32F(cv::Mat& img);

        /**
        * Read a Fits file in 16 bits unsigned char format.
        * @param img Reference on the container which will contain the read fits.
        *
        */
        bool readFits16US(cv::Mat& img);

        /**
        * Read a Fits file in 16 bits signed char format.
        * @param img Reference on the container which will contain the read fits.
        *
        */
        bool readFits16S(cv::Mat& img);

        /**
        * Read a Fits file in 8 bits unsigned char format.
        * @param img Reference on the container which will contain the read fits.
        *
        */
        bool readFits8UC(cv::Mat& img);

        /**
        * Read a Fits file in 8 bits signed char format.
        * @param img Reference on the container which will contain the read fits.
        *
        */
        bool readFits8C(cv::Mat& img);

        /**
        * Read a keyword in integer type.
        * @param keyword Keyword name.
        * @param value Reference on the found keyword's value.
        *
        */
        bool readIntKeyword(std::string keyword, int& value);

        /**
        * Read a keyword in string type.
        * @param keyword Keyword name.
        * @param value Reference on the found keyword's value.
        *
        */
        bool readStringKeyword(std::string keyword, std::string& value);

        /**
        * Read a keyword in double type.
        * @param keyword Keyword name.
        * @param value Reference on the found keyword's value.
        *
        */
        bool readDoubleKeyword(std::string keyword, double& value);

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
        bool writeKeywords(fitsfile* fptr);

    };
}
