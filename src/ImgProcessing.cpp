/*
                            ImgProcessing.cpp

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
* \file    ImgProcessing.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
*/

#include "ImgProcessing.h"
using namespace freeture;

cv::Mat ImgProcessing::correctGammaOnMono8(cv::Mat& img, double gamma) {

    double gammaInverse = 1.0 / gamma;

    cv::Mat lutMatrix(1, 256, CV_8UC1 );
    uchar * ptr = NULL;
    ptr = lutMatrix.ptr();
    for( int i = 0; i < 256; i++ )
    ptr[i] = (int)( pow( (double) i / 255.0, gammaInverse ) * 255.0 );

    cv::Mat result;
    LUT( img, lutMatrix, result );

    return result;

}

cv::Mat ImgProcessing::correctGammaOnMono12(cv::Mat& img, double gamma) {

    double gammaInverse = 1.0 / gamma;

    cv::Mat result = cv::Mat(img.rows, img.cols, CV_16UC1, cv::Scalar(0));

    unsigned short * ptr = NULL;
    unsigned short * ptr2 = NULL;

    for(int i = 0; i < img.rows; i++){

        ptr = img.ptr<unsigned short>(i);
        ptr2 = result.ptr<unsigned short>(i);

        for(int j = 0; j < img.cols; j++){

            ptr2[j] = (int)( pow( (double) ptr[j] / 4095.0, gammaInverse ) * 4095.0 );

        }

    }

    return result;

}

cv::Mat ImgProcessing::buildSaturatedMap(cv::Mat &img, int maxval) {

    cv::Mat saturatedMap = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar(0));

    if(img.type() == CV_16UC1) {

        unsigned short * ptr1;
        unsigned char * ptr2;

        for(int i = 0; i < img.rows; i++) {

            ptr1 = img.ptr<unsigned short>(i);
            ptr2 = saturatedMap.ptr<unsigned char>(i);

            for(int j = 0; j < img.cols; j++){

                if(ptr1[j] >= maxval) {
                    ptr2[j] = 255;
                }
            }
        }
    }

    if(img.type() == CV_8UC1) {

        unsigned char * ptr1;
        unsigned char * ptr2;

        for(int i = 0; i < img.rows; i++) {

            ptr1 = img.ptr<unsigned char>(i);
            ptr2 = saturatedMap.ptr<unsigned char>(i);

            for(int j = 0; j < img.cols; j++){

                if(ptr1[j] >= maxval) {
                    ptr2[j] = 255;
                }
            }
        }
    }

    return saturatedMap;

}

cv::Mat ImgProcessing::thresholding(cv::Mat &img, cv::Mat &mask, int factor, Thresh threshType) {

    cv::Mat thresholdedMap = cv::Mat(img.rows,img.cols, CV_8UC1, cv::Scalar(0));
    cv::Scalar mean, stddev;
    cv::meanStdDev(img, mean, stddev, mask);
    int threshold = 0;

    switch(threshType) {

        case Thresh::MEAN :

            threshold = mean[0] * factor;

            break;

        case Thresh::STDEV :

            threshold = stddev[0] * factor;

            break;

    }

    switch(img.type()) {

        case CV_16UC1 :
            {
                if(threshold == 0)
                    threshold = 65535;

                unsigned short * ptr1;
                unsigned char * ptrMap;

                for(int i = 0; i < img.rows; i++) {

                    ptr1 = img.ptr<unsigned short>(i);
                    ptrMap = thresholdedMap.ptr<unsigned char>(i);

                    for(int j = 0; j < img.cols; j++){

                        if(ptr1[j] > threshold) {
                            ptrMap[j] = 255;
                        }
                    }
                }
            }

            break;

        case CV_8UC1 :

            {
                if(threshold == 0)
                    threshold = 255;

                unsigned char * ptr1;
                unsigned char * ptrMap;

                for(int i = 0; i < img.rows; i++) {

                    ptr1 = img.ptr<unsigned char>(i);
                    ptrMap = thresholdedMap.ptr<unsigned char>(i);

                    for(int j = 0; j < img.cols; j++){

                        if(ptr1[j] > threshold) {
                            ptrMap[j] = 255;
                        }
                    }
                }
            }
            break;

    }

    return thresholdedMap;

}

// Create n * n region in a frame ( n is a pair value)
void ImgProcessing::subdivideFrame(std::vector<cv::Point> &sub, int n, int imgH, int imgW) {

    /*

    Example : frame with n = 4 -> 16 subdivisions returned

    |07|08|09|10|
    |06|01|02|11|
    |05|04|03|12|
    |16|15|14|13|

    */

    int subW = imgW/n;
    int subH = imgH/n;

    cv::Point first = cv::Point((n/2 - 1) * subW, (n/2)*subH);
    cv::Point last = cv::Point(imgW - subW, imgH - subH);

    sub.push_back(first);

    int x = first.x, y = first.y;
    int nbdep = 0,
        nbdepLimit = 1,
        dep = 1; // 1 up
                 // 2 right
                 // 3 down
                 // 4 left


    for(int i = 1; i < n * n; i++){

        if(dep == 1){

            y = y - subH;
            sub.push_back(cv::Point(x,y));
            nbdep ++;
            if(nbdep == nbdepLimit){
                nbdep = 0;
                dep ++;
            }

        }else if(dep == 2){

            x = x + subW;
            sub.push_back(cv::Point(x,y));
            nbdep ++;
            if(nbdep == nbdepLimit){
                nbdep = 0;
                nbdepLimit++;
                dep ++;
            }

        }else if(dep == 3){

            y = y + subH;
            sub.push_back(cv::Point(x,y));
            nbdep ++;
            if(nbdep == nbdepLimit){
                nbdep = 0;
                dep ++;
            }

        }else if(dep == 4){

            x = x - subW;
            sub.push_back(cv::Point(x,y));
            nbdep ++;
            if(nbdep == nbdepLimit){
                nbdep = 0;
                nbdepLimit++;
                dep = 1;
            }
        }
    }
}

cv::Mat ImgProcessing::subdivideFrame(cv::Mat img, int n) {

    std::vector<cv::Point> listSubPos;

    int subW = img.cols/n;
    int subH = img.rows/n;

    std::cout << "subW : " << subW << std::endl;
    std::cout << "subH : " << subH << std::endl;

    for(int j = 0; j < n; j++) {

        for(int i = 0; i < n; i++) {

            listSubPos.push_back(cv::Point(i*subW, j*subH));
            // cout << Point(i*subW, j*subH)<< endl;

        }

    }

    cv::Mat imgSubdivided;
    img.copyTo(imgSubdivided);

    for(int i = 0; i < n; i++)
        line(imgSubdivided, cv::Point(i * subW, 0), cv::Point(i*subW, subH * n), cv::Scalar(255), 1, 8);

    for(int j = 0; j < n; j++)
        line(imgSubdivided, cv::Point(0, j * subH), cv::Point(subW * n, j * subH), cv::Scalar(255), 1, 8);

    return imgSubdivided;

}
