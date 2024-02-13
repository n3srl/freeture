#pragma once
/*
                              ImgProcessing.h

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
* \file    ImgProcessing.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
*/
#include "Commons.h"

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace freeture
{
    enum class Thresh {

        MEAN,
        STDEV

    };

    class ImgProcessing {

    public:

        /**
        * Gamma correction on Mono8 image..
        *
        * @param img Opencv mat image to correct.
        * @param gamma Gamma value.
        * @return Image with gamma corrected.
        */
        static cv::Mat correctGammaOnMono8(cv::Mat& img, double gamma);

        /**
        * Gamma correction on Mono12 image.
        *
        * @param img Opencv mat image to correct.
        * @param gamma Gamma value.
        * @return Image with gamma corrected.
        */
        static cv::Mat correctGammaOnMono12(cv::Mat& img, double gamma);

        static cv::Mat buildSaturatedMap(cv::Mat& img, int maxval);

        static cv::Mat thresholding(cv::Mat& img, cv::Mat& mask, int factor, Thresh threshType);

        // Subdivise a frame in n regions and fetch their coordinates.
        static void subdivideFrame(std::vector<cv::Point>& sub, int n, int imgH, int imgW);

        // Subdivise a frame in n regions and draw them.
        static cv::Mat subdivideFrame(cv::Mat img, int n);


    };
}
