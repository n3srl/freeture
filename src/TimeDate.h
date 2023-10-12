/*
                                TimeDate.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture

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
* \file    TimeDate.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    13/06/2014
* \brief   Time helpers.
*/

#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <boost/tokenizer.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <list>
#include "Conversion.h"


using namespace boost::posix_time;

class TimeDate {

    public :

        struct Date {

            int     year;
            int     month;
            int     day;
            int     hours;
            int     minutes;
            double  seconds;

            Date():year(0), month(0), day(0), hours(0), minutes(0), seconds(0) {}

        };

        /**
        * Get local date time in UT
        *
        * @param pt Boost ptime.
        * @param format "%Y:%m:%d:%H:%M:%S"
        * @return Date time in string.
        */
        static std::string localDateTime(::boost::posix_time::ptime pt, std::string format);

        /**
        * Get date in UT
        *
        * @return YYYY, MM, DD.
        */
        static std::string getCurrentDateYYYYMMDD();

        /**
        * Convert gregorian date to julian date.
        *
        * @param date Vector of date : YYYY, MM, DD, hh, mm, ss.
        * @return Julian date.
        */
        static double gregorianToJulian(Date date);

        /**
        * Get julian century from julian date.
        *
        * @param julianDate.
        * @return Julian century.
        */
        static double julianCentury(double julianDate);

        /**
        * Convert Hours:Minutes:seconds to decimal hours.
        *
        * @param H hours.
        * @param M minutes.
        * @param S seconds.
        * @return decimal hours.
        */
        static double hmsToHdecimal(int H, int M, int S);

        /**
        * Convert decimal hours to Hours:Minutes:seconds.
        *
        * @param val Decimal hours.
        * @return Vector of Hours:Minutes:seconds.
        */
        static std::vector<int> HdecimalToHMS(double val);

        /**
        * Get local sideral time.
        *
        * @param julianCentury
        * @param gregorianH
        * @param gregorianMin
        * @param gregorianS
        * @param longitude
        * @return Local sideral time.
        */
        static double localSideralTime_2(double julianCentury, int gregorianH, int gregorianMin, int gregorianS, double longitude);

        /**
        * Get local sideral time.
        *
        * @param julianDate
        * @param gregorianH
        * @param gregorianMin
        * @param gregorianS
        * @return Local sideral time.
        */
        static double localSideralTime_1(double JD0, int gregorianH, int gregorianMin, int gregorianS);

        /**
        * Split string to int values according to the ":" separator.
        *
        * @param str String to split. Its format is YYYY:MM:DD:HH:MM:SS
        * @return Vector of int values.
        */
        static std::vector<int> splitStringToInt(std::string str);

        /**
        * Get YYYYMMDD from date string.
        *
        * @param date Date with the following format : YYYY-MM-DDTHH:MM:SS.fffffffff
        * @return YYYYMMDD.
        */
        static std::string getYYYYMMDDfromDateString(std::string date);

        // output : YYYYMMDD
        static std::string getYYYYMMDD(Date date);

        /**
        * Get year, month, date, hours, minutes and seconds from date string.
        *
        * @param date Date with the following format : YYYY-MM-DDTHH:MM:SS.fffffffff
        * @return Vector of int values.
        */
        static std::vector<int> getIntVectorFromDateString(std::string date);

        /**
        * Get YYYYMMJJTHHMMSS date.
        *
        * @param date YYYYMMJJTHHMMSS.fffffffff
        * @return YYYYMMJJTHHMMSS.
        */
        static std::string getYYYYMMDDThhmmss(std::string date);

        /**
        * Get YYYYMMJJTHHMMSS date.
        *
        * @param date YYYYMMJJTHHMMSS.fffffffff
        * @return YYYYMMJJTHHMMSS.
        */
        static std::string getYYYYMMDDThhmmss(Date date);

        static Date splitIsoExtendedDate(std::string date);

        static std::string getIsoExtendedFormatDate(Date date);

        /**
        * Get seconds between two dates
        */
        static int secBetweenTwoDates(Date d1, Date d2);

};

