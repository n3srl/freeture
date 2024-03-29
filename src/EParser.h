#pragma once
/*
                                EParser.h

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
*   Last modified:      04/12/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    EParser.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    04/12/2014
* \brief   Parse some parameters in the configuration file with FreeTure's enumerations.
*/


#include <string>
#include <iostream>
#include <map>
#include <stdexcept>

//http://stackoverflow.com/questions/726664/string-to-enum-in-c
namespace freeture
{
    template<typename T> class EParser {

    public:

        std::map<std::string, T> enumMap;     // String value with its enumeration value.

    public:

        /**
        * Constructor.
        *
        */
        EParser();

        /**
        * Get enumeration value from its string version.
        *
        * @param paramName Name of the parameter which have an enumeration value.
        * @param value Enumeration value in string.
        * @return Enumeration value.
        */
        T parseEnum(std::string paramName, const std::string& value) {

            typename std::map<std::string, T>::const_iterator iValue = enumMap.find(value);

            if (iValue == enumMap.end()) {

                typename std::map<std::string, T>::const_iterator it;

                std::string res = "<" + value + "> is not correct. Available values are : \n";

                for (it = enumMap.begin(); it != enumMap.end(); ++it) {
                    res = res + "    <" + it->first + ">\n";
                }

                throw std::runtime_error(res);

            }

            return iValue->second;

        }

        T parseEnum(const std::string value) {

            typename std::map<std::string, T>::const_iterator iValue = enumMap.find(value);

            if (iValue == enumMap.end()) {

                throw "Enum not found";

            }

            return iValue->second;

        }

        bool isEnumValue(const std::string value) {

            typename std::map<std::string, T>::const_iterator iValue = enumMap.find(value);

            if (iValue == enumMap.end()) {
                return false;
            }

            return true;

        }

        /**
        * Get string value of enumeration.
        *
        * @param type Enumeration value.
        * @return String of enumeration.
        */
        std::string getStringEnum(T type) {

            typename std::map<std::string, T>::const_iterator it;

            for (it = enumMap.begin(); it != enumMap.end(); ++it) {
                if (type == it->second)
                    return it->first;
            }

            return "";
        }

    };
}