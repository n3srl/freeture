/*
                            Conversion.cpp

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
* \file    Conversion.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    13/06/2014
* \brief   Various conversion tools.
*/

#include "Conversion.h"

std::string Conversion::matTypeToString(int type) {

    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {

        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;

    }

    r += "C";
    r += (chans+'0');

    return r;

}

std::string Conversion::intToString(int nb){

    std::ostringstream oss;
    oss << nb;
    std::string result = oss.str();
    return result;

}

float Conversion::roundToNearest(float value, float precision) {

    float fractpart1 = 0.0, intpart1 = 0.0, fractpart2 = 0.0, intpart2 = 0.0;
    fractpart1 = modf (value , (double*)&intpart1);
    float d = fractpart1/precision;
    fractpart2 = modf (d , (double*)&intpart2);
    return intpart1 + intpart2*precision;
}

std::string Conversion::floatToString(float nb){

    std::ostringstream ss;
    ss << nb;
    std::string s(ss.str());
    return s;

}

std::string Conversion::doubleToString(double nb) {

    std::ostringstream strs;
    strs << nb;
    std::string str = strs.str();

    return str;

}

void Conversion::stringTok(std::list<std::string> &container, std::string const &in, const char * const delimiters  = "_"){

    const std::string::size_type len = in.length();
    std::string::size_type i = 0;

    while (i < len){

        // Eat leading whitespace
        i = in.find_first_not_of(delimiters, i);

        if (i == std::string::npos)
            return;   // Nothing left but white space

        // Find the end of the token
        std::string::size_type j = in.find_first_of(delimiters, i);

        // Push token
        if (j == std::string::npos){

            container.push_back(in.substr(i));
            return;

        }else

            container.push_back(in.substr(i, j-i));

        // Set up for next loop
        i = j + 1;

    }
}

Mat Conversion::convertTo8UC1(Mat &img) {

    Mat tmp;
    img.copyTo(tmp);
    double min, max;
    minMaxLoc(tmp, &min, &max);
    tmp.convertTo(tmp, CV_8UC1, 255.0/(max - min), -min * 255.0/(max - min));

    return tmp;

}

int Conversion::countNumberDigit(int n){

    int nbDigit = 0;

    while(n!=0){

      n/=10;
      ++nbDigit;

    }

    return nbDigit;
}

int Conversion::roundToUpperRange(int n) {

    int nbDigit = 0;
    int last = 0;

    while(n!=0){

        last =  n;
        n/=10;
        ++nbDigit;

    }

    int f = 1;

    for(int i = 1; i < nbDigit; i++)
        f *=10;

    return (last+1) * f;
}

std::string Conversion::numbering(int totalDigit, int n) {

    int cpt = 0;

    int nbZeroToAdd = 0;

    std::string ch = "";

    if(n<10){

        nbZeroToAdd = totalDigit - 1;

        for(int i = 0; i < nbZeroToAdd; i++){

            ch += "0";

        }

    }else{

        while(n > 0){

            n/=10;
            cpt ++;

        }

        nbZeroToAdd = totalDigit - cpt;

        for(int i = 0; i < nbZeroToAdd; i++){

            ch += "0";

        }

    }

    return ch;
}
