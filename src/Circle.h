#pragma once
/*
                                Circle.h

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
* \file    Circle.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    19/06/2014
* \brief
*/
//header refactoring ok
#include "Commons.h"

#include <iostream>

#include <opencv2/opencv.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Conversion.h"
#include "SaveImg.h"

namespace freeture
{
    class Circle {

    private :

        cv::Point   mPos;     // Center position.
        float       mRadius;

    public:

        Circle(cv::Point center, float radius) :mPos(center), mRadius(radius) {

        }

        cv::Point getCenter() { return mPos; };
        float getRadius() { return mRadius; };

        bool computeDiskSurfaceIntersection(Circle c2,
            float& surfaceCircle1,
            float& surfaceCircle2,
            float& intersectedSurface,
            bool enableDebug,
            std::string debugPath) {

            cv::Mat map;
            bool res = false;
            bool displayIntersectedSurface = false;

            if(enableDebug) map = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0,0,0));


            surfaceCircle1 = 0.0;
            surfaceCircle2 = 0.0;
            intersectedSurface = 0.0;


            if(enableDebug) circle(map, mPos, (int)mRadius, cv::Scalar(0,255,0));
            if(enableDebug) circle(map, c2.getCenter(), (int)c2.getRadius(), cv::Scalar(0,0,255));


            // Distance between two circles centers
            double distPcNc = sqrt(pow((mPos.x - c2.getCenter().x), 2) + pow((mPos.y - c2.getCenter().y), 2));

            // No intersections.
            if (distPcNc > c2.getRadius() + mRadius) {


                if(enableDebug) putText(map, "No intersections." , cv::Point(15,15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, cv::LINE_AA);

                res = false;

                // Circles coincide.
            }
            else if (distPcNc == 0 && c2.getRadius() == mRadius) {

                if(enableDebug) putText(map, "Circles coincides." , cv::Point(15,15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, cv::LINE_AA);

                res = true;

                // A circle is contained inside the other.
            }
            else if (distPcNc < abs(c2.getRadius() - mRadius)) {

                if(enableDebug) putText(map, "A circle is contained whithin the other." , cv::Point(15,15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, cv::LINE_AA);

                res = true;

            }
            else {

                surfaceCircle1 = M_PI * pow(mRadius, 2);
                surfaceCircle2 = M_PI * pow(c2.getRadius(), 2);

                float R0 = mRadius;
                float R1 = c2.getRadius();
                double x0 = mPos.x;
                double y0 = mPos.y;
                double x1 = c2.getCenter().x;
                double y1 = c2.getCenter().y;

                if (mPos.y != c2.getCenter().y) {

                    double N = (pow(R1, 2) - pow(R0, 2) - pow(x1, 2) + pow(x0, 2) - pow(y1, 2) + pow(y0, 2)) / (2 * (y0 - y1));
                    double A = pow((x0 - x1) / (y0 - y1), 2) + 1;
                    double B = 2 * y0 * ((x0 - x1) / (y0 - y1)) - 2 * N * ((x0 - x1) / (y0 - y1)) - 2 * x0;
                    double C = pow(x0, 2) + pow(y0, 2) + pow(N, 2) - pow(R0, 2) - 2 * y0 * N;
                    double delta = std::sqrt(pow(B, 2) - 4 * A * C);

                    //cout << delta << endl;

                    if (delta > 0) {

                        double resX1 = (-B - delta) / (2 * A);
                        double resX2 = (-B + delta) / (2 * A);

                        double resY1 = N - resX1 * ((x0 - x1) / (y0 - y1));
                        double resY2 = N - resX2 * ((x0 - x1) / (y0 - y1));

                        if(enableDebug) line(map, cv::Point((int)resX1, (int)resY1 ), cv::Point((int)resX2, (int)resY2 ), cv::Scalar(255,255,255), 1, cv::LINE_AA);


                        // Circle1 more inside the other
                        if (distPcNc > abs(c2.getRadius() - mRadius) && distPcNc < c2.getRadius() && c2.getRadius() > mRadius) {

                            //cout << "one circle more inside the other" << endl;

                            // Cord length.
                            double c = sqrt(pow((resX1 - resX2), 2) + pow((resY1 - resY2), 2));
                            double cc = c / (2.0 * R0);
                            if (cc > 1.0) cc = 1.0;
                            double thetaCircle1 = 2.0 * asin(cc);
                            double areaCircle1 = (pow(R0, 2) / 2) * (thetaCircle1 - sin(thetaCircle1));

                            double ccc = c / (2.0 * R1);
                            if (ccc > 1.0) ccc = 1.0;
                            double thetaCircle2 = 2 * asin(ccc);
                            double areaCircle2 = (pow(R1, 2) / 2) * (thetaCircle2 - sin(thetaCircle2));

                            intersectedSurface = surfaceCircle1 - areaCircle1 + areaCircle2;

                            displayIntersectedSurface = true;

                            // Circle2 more inside the other
                        }
                        else if (distPcNc > abs(c2.getRadius() - mRadius) && distPcNc < mRadius && mRadius > c2.getRadius()) {

                            //cout << "one circle more inside the other" << endl;

                            // Cord length.
                            double c = sqrt(pow((resX1 - resX2), 2) + pow((resY1 - resY2), 2));

                            double cc = c / (2.0 * R0);
                            if (cc > 1.0) cc = 1.0;
                            double thetaPosCircle = 2.0 * asin(cc);
                            double areaPosCircle = (pow(R0, 2) / 2) * (thetaPosCircle - sin(thetaPosCircle));

                            double ccc = c / (2.0 * R1);
                            if (ccc > 1.0) ccc = 1.0;
                            double thetaNegCircle = 2 * asin(ccc);
                            double areaNegCircle = (pow(R1, 2) / 2) * (thetaNegCircle - sin(thetaNegCircle));

                            intersectedSurface = surfaceCircle2 - areaNegCircle + areaPosCircle;

                            displayIntersectedSurface = true;

                        }
                        else if (distPcNc == c2.getRadius() || distPcNc == mRadius) {

                            //cout << "Outskirt" << endl;

                        }
                        else {

                            double c = sqrt(pow((resX1 - resX2), 2) + pow((resY1 - resY2), 2));

                            double cc = c / (2.0 * R0);
                            if (cc > 1.0) cc = 1.0;
                            double thetaPosCircle = 2.0 * asin(cc);
                            double areaPosCircle = (pow(R0, 2) / 2) * (thetaPosCircle - sin(thetaPosCircle));

                            double ccc = c / (2.0 * R1);
                            if (ccc > 1.0) ccc = 1.0;
                            double thetaNegCircle = 2 * asin(ccc);
                            double areaNegCircle = (pow(R1, 2) / 2) * (thetaNegCircle - sin(thetaNegCircle));

                            intersectedSurface = areaNegCircle + areaPosCircle;

                            displayIntersectedSurface = true;

                        }

                        res = true;

                    }

                }
                else {

                    double x = (pow(R1, 2) - pow(R0, 2) - pow(x1, 2) + pow(x0, 2)) / (2 * (x0 - x1));
                    double A = 1.0;
                    double B = -2 * y1;
                    double C = pow(x1, 2) + pow(x, 2) - 2 * x1 * x + pow(y1, 2) - pow(R1, 2);

                    double delta = std::sqrt(pow(B, 2) - 4 * A * C);

                    if (delta > 0) {

                        double resY1 = (-B - delta) / (2 * A);
                        double resY2 = (-B + delta) / (2 * A);

                        double resX1 = (pow(R1, 2) - pow(R0, 2) - pow(x1, 2) + pow(x0, 2) - pow(y1, 2) + pow(y0, 2)) / (2 * (x0 - x1));
                        double resX2 = (pow(R1, 2) - pow(R0, 2) - pow(x1, 2) + pow(x0, 2) - pow(y1, 2) + pow(y0, 2)) / (2 * (x0 - x1));

                        if(enableDebug) line(map, cv::Point((int)resX1,(int)resY1 ), cv::Point((int)resX2, (int)resY2 ), cv::Scalar(255,255,255), 1, cv::LINE_AA);


                        // Circle neg more inside the other
                        if (distPcNc > abs(c2.getRadius() - mRadius) && distPcNc < c2.getRadius() && c2.getRadius() > mRadius) {

                            // Cord length.
                            double c = sqrt(pow((resX1 - resX2), 2) + pow((resY1 - resY2), 2));

                            double cc = c / (2.0 * R0);
                            if (cc > 1.0) cc = 1.0;
                            double thetaPosCircle = 2.0 * asin(cc);

                            double areaPosCircle = (pow(R0, 2) / 2) * (thetaPosCircle - sin(thetaPosCircle));

                            double ccc = c / (2.0 * R1);
                            if (ccc > 1.0) ccc = 1.0;
                            double thetaNegCircle = 2 * asin(ccc);
                            double areaNegCircle = (pow(R1, 2) / 2) * (thetaNegCircle - sin(thetaNegCircle));

                            intersectedSurface = surfaceCircle1 - areaPosCircle + areaNegCircle;

                            displayIntersectedSurface = true;

                            // Circle pos more inside the other
                        }
                        else if (distPcNc > abs(c2.getRadius() - mRadius) && distPcNc < mRadius && mRadius > c2.getRadius()) {

                            // Cord length.
                            double c = sqrt(pow((resX1 - resX2), 2) + pow((resY1 - resY2), 2));

                            double cc = c / (2.0 * R0);
                            if (cc > 1.0) cc = 1.0;
                            double thetaPosCircle = 2.0 * asin(cc);
                            double areaPosCircle = (pow(R0, 2) / 2) * (thetaPosCircle - sin(thetaPosCircle));

                            double ccc = c / (2.0 * R1);
                            if (ccc > 1.0) ccc = 1.0;
                            double thetaNegCircle = 2 * asin(ccc);
                            double areaNegCircle = (pow(R1, 2) / 2) * (thetaNegCircle - sin(thetaNegCircle));

                            intersectedSurface = surfaceCircle2 - areaNegCircle + areaPosCircle;

                            displayIntersectedSurface = true;

                        }
                        else if (distPcNc == c2.getRadius() || distPcNc == mRadius) {

                            //cout << "Le centre d'un des cercles est sur la p�riph�rie de l'autre" << endl;

                        }
                        else {

                            double c = sqrt(pow((resX1 - resX2), 2) + pow((resY1 - resY2), 2));

                            double cc = c / (2.0 * R0);
                            if (cc > 1.0) cc = 1.0;
                            double thetaPosCircle = 2.0 * asin(cc);
                            double areaPosCircle = (pow(R0, 2) / 2) * (thetaPosCircle - sin(thetaPosCircle));

                            double ccc = c / (2.0 * R1);
                            if (ccc > 1.0) ccc = 1.0;
                            double thetaNegCircle = 2 * asin(ccc);
                            double areaNegCircle = (pow(R1, 2) / 2) * (thetaNegCircle - sin(thetaNegCircle));

                            intersectedSurface = areaNegCircle + areaPosCircle;

                            displayIntersectedSurface = true;


                        }

                        res = true;

                    }

                }

            }

            if (enableDebug && displayIntersectedSurface) {

                putText(map, "Intersected surface : " , cv::Point(15,15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, cv::LINE_AA);
                std::string msg1 = "- Green circle : " + Conversion::floatToString((intersectedSurface * 100) / surfaceCircle1) + "%" ;
                putText(map, msg1 , cv::Point(15,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, cv::LINE_AA);
                std::string msg2 = "- Red circle : " + Conversion::floatToString((intersectedSurface * 100) / surfaceCircle2)+ "%";
                putText(map, msg2 , cv::Point(15,45), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, cv::LINE_AA);

            }

            if (enableDebug) SaveImg::saveBMP(map, debugPath);

            return res;

        }

    };
}