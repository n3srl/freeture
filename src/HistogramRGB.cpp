/*
                            HistogramRGB.cpp

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
* \file    HistogramRGB.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
* \brief   Create/Analyse histogram of a rgb image.
*/

#include "HistogramRGB.h"
using namespace freeture;

HistogramRGB::HistogramRGB(){

    bins = cv::Mat( 3, 256, CV_32F, cv::Scalar( 0.f ) );

}

int HistogramRGB::calculate(cv::Mat& image){

    if( image.type() != CV_8UC3 ) { return 1; }

    this->clear();

    cv::Mat_< cv::Vec3b >::iterator it = image.begin< cv::Vec3b >();
    cv::Mat_< cv::Vec3b >::iterator itend = image.end< cv::Vec3b >();

    for( ; it != itend; ++it ){
        bins.at< float >( 0, (*it)[0] ) += 1.f;
        bins.at< float >( 1, (*it)[1] ) += 1.f;
        bins.at< float >( 2, (*it)[2] ) += 1.f;
    }

    return 0;
}

void HistogramRGB::normalize(void){

    cv::Mat_< float >::iterator it = bins.begin< float >();
    cv::Mat_< float >::iterator itend = bins.end< float >();

    float max = 1.f;

    for( ; it != itend; ++it ){
        if( *it > max ) max = *it;
    }

    bins /= max;
}

cv::Mat HistogramRGB::render(void){

    int w_hist = 256;
    int h_hist = 100;

    cv::Mat result( h_hist, w_hist, CV_8UC3, cv::Scalar( 0 ) );
    cv::Point start( 0, h_hist - 1 ), end( 0, h_hist - 1 );

    for( int i = 0; i < w_hist; i++ ){
        start.x = end.x = i;

        end.y = h_hist - cvRound( h_hist * bins.at< float >( 0, i ) );
        line( result, start, end, cv::Scalar( 150, 40, 40 ) );

        end.y = h_hist - cvRound( h_hist * bins.at< float >( 1, i ) );
        line( result, start, end, cv::Scalar( 40, 150, 40 ) );

        end.y = h_hist - cvRound( h_hist * bins.at< float >( 2, i ) );
        line( result, start, end, cv::Scalar( 40, 40, 160 ) );
    }

    for( int i = 1; i < w_hist; i++ ){
        /// Contour line

        line( result, cv::Point( i - 1,   h_hist - cvRound( h_hist * bins.at< float >(0, i - 1    ) ) ),
            cv::Point(i, h_hist - cvRound(h_hist * bins.at< float >(0, i))),
            cv::Scalar(255, 0, 0));

        line( result, cv::Point( i - 1,   h_hist - cvRound( h_hist * bins.at< float >(1, i - 1    ) ) ),
            cv::Point( i,       h_hist - cvRound( h_hist * bins.at< float >(1, i        ) ) ),
            cv::Scalar( 0, 255, 0) );

        line( result, cv::Point( i - 1,   h_hist - cvRound( h_hist * bins.at< float >(2, i - 1    ) ) ),
            cv::Point( i,       h_hist - cvRound( h_hist * bins.at< float >(2, i        ) ) ),
            cv::Scalar( 0, 0, 255) );
    }

    return result;
}

