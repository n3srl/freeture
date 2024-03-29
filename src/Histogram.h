#pragma once
/*
                            Histogram.h

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
* \file    Histogram.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
* \brief   Create/Analyse histogram of a given image.
*/
#include "Commons.h"


#include <opencv2/opencv.hpp>
#include <iostream>

namespace freeture
{
    class Histogram {

    protected:

        cv::Mat bins;

    public:

        /**
        * Constructor.
        *
        */
        Histogram() {};

        /**
        * Destructor.
        *
        */
        ~Histogram() {};

        /**
        * Calculate number of pixels in each bins.
        *
        * @param image Opencv mat image to analysis.
        * @return Success status.
        */
        virtual int calculate(cv::Mat& image) = 0;

        /**
        * Normalize bins.
        *
        */
        virtual void normalize(void) = 0;

        /**
        * Build histogram image.
        *
        * @return Histogram.
        */
        virtual cv::Mat render(void) = 0;

    };
}
