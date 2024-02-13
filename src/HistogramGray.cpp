/*
                            HistogramGray.cpp

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
* \file    HistogramGray.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
* \brief   Create/Analyse histogram of a grey image.
*/

#include "HistogramGray.h"

using namespace freeture;

HistogramGray::HistogramGray(){

        bins = cv::Mat(1, 256, CV_32F, cv::Scalar(0.f));

}

int HistogramGray::calculate(cv::Mat& image){

    if( image.channels() != 1 || image.type() != CV_8U ) { return 1; }
    this->clear();
    cv::Mat_< uchar >::iterator it = image.begin< uchar >();
    cv::Mat_< uchar >::iterator itend = image.end< uchar >();

    for( ; it != itend; ++it ) {
        bins.at< float >( 0, *it ) += 1.f;
    }

    return 0;

}

void HistogramGray::normalize(void){

    cv::Mat_< float >::iterator it = bins.begin< float >();
    cv::Mat_< float >::iterator itend = bins.end< float >();
    float max = 1.f;

    for( ; it != itend; ++it ) {
        if( *it > max ) max = *it;
    }

    bins /= max;
}

cv::Mat HistogramGray::render(void){

    cv::Mat result( 100, 256, CV_8U, cv::Scalar( 0 ) );
    cv::Point start( 0, 0 ), end( 0, 0 );

    for( int i = 0; i < 256; i++ ) {
        start.x = end.x = i;
        end.y = cvRound( 100.f * bins.at< float >( i ) );
        line( result, start, end, cv::Scalar( 255 ) );
    }

    flip( result, result, 0 );

    return result;
}

cv::Mat HistogramGray::renderHistogramOnImage(cv::Mat image){

    cv::Mat h = render();

    h.copyTo(image(cv::Rect(0, image.rows - h.rows,h.cols,h.rows)));

    return image;

}
