/*
                                EParser.cpp

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
* \file    EParser.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    04/12/2014
* \brief   Parse some parameters in the configuration file with FreeTure's enumerations.
*/

#include "EParser.h"

#include "EDetMeth.h"
#include "EStackMeth.h"
#include "ELogSeverityLevel.h"
#include "ESmtpSecurity.h"
#include "ETimeMode.h"
#include "EImgFormat.h"
#include "ECamPixFmt.h"
#include "EInputDeviceType.h"
#include "ECamSdkType.h"
#include "EFreetureMode.h"

using namespace freeture;

template<> EParser<CamPixFmt>::EParser() 
{

    enumMap["MONO8"]   = CamPixFmt::MONO8;
    enumMap["GREY"]    = CamPixFmt::GREY;
    enumMap["Y800"]    = CamPixFmt::Y800;
    enumMap["MONO12"]  = CamPixFmt::MONO12;
    enumMap["YUYV"]    = CamPixFmt::YUYV;
    enumMap["UYVY"]    = CamPixFmt::UYVY;
    enumMap["RGB565"]  = CamPixFmt::RGB565;
    enumMap["BGR3"]    = CamPixFmt::BGR3;
    enumMap["RGB3"]    = CamPixFmt::RGB3;
    enumMap["MONO16"]  = CamPixFmt::MONO16;
    enumMap["*** NOT DEFINED ***"] = CamPixFmt::UNDEFINED;

}

template<> EParser<SmtpSecurity>::EParser() {

    enumMap["NO_SECURITY"]  = NO_SECURITY;
    enumMap["USE_TLS"]      = USE_TLS;
    enumMap["USE_SSL"]      = USE_SSL;

}

template<> EParser<InputDeviceType>::EParser() {

    enumMap["CAMERA"]               = CAMERA;
    enumMap["VIDEO"]                = VIDEO;
    enumMap["UNDEFINED_INPUT_TYPE"] = UNDEFINED_INPUT_TYPE;
    enumMap["SINGLE_FITS_FRAME"]    = SINGLE_FITS_FRAME;

}

template<> EParser<StackMeth>::EParser() {

    enumMap["SUM"]      = SUM;
    enumMap["MEAN"]     = MEAN;

}

template<> EParser<DetMeth>::EParser() {

    enumMap["TEMPORAL_MTHD"]    = TEMPORAL_MTHD;
    enumMap["TEMPLATE_MTHD"]    = TEMPLATE_MTHD;

}

template<> EParser<LogSeverityLevel>::EParser() {

    enumMap["normal"]           = LogSeverityLevel::normal;
    enumMap["notification"]     = LogSeverityLevel::notification;
    enumMap["fail"]             = LogSeverityLevel::fail;
    enumMap["warning"]          = LogSeverityLevel::warning;
    enumMap["critical"]         = LogSeverityLevel::critical;

}

template<> EParser<TimeMode>::EParser() {

    enumMap["DAY"]          = DAY;
    enumMap["NIGHT"]        = NIGHT;
    enumMap["DAYNIGHT"]     = DAYNIGHT;

}

template<> EParser<ImgFormat>::EParser() {

    enumMap["JPEG"]        = JPEG;
    enumMap["FITS"]        = FITS;

}

template<> EParser<CamSdkType>::EParser() {

    enumMap["ARAVIS"]        = CamSdkType::ARAVIS;
    enumMap["LUCID_ARENA"]   = CamSdkType::LUCID_ARENA;
    enumMap["PYLONGIGE"]     = CamSdkType::PYLONGIGE;
    enumMap["TIS"]           = CamSdkType::TIS;
    enumMap["VIDEOFILE"]     = CamSdkType::VIDEOFILE;
    enumMap["FRAMESDIR"]     = CamSdkType::FRAMESDIR;
    enumMap["V4L2"]          = CamSdkType::V4L2;
    enumMap["VIDEOINPUT"]    = CamSdkType::VIDEOINPUT;
    enumMap["UNKNOWN"]       = CamSdkType::UNKNOWN;
}


template<> EParser<FreetureMode>::EParser() {
    enumMap["Test configuration"] = FreetureMode::TEST_CONFIGURATION;
    enumMap["Print help"] = FreetureMode::PRINT_HELP;
    enumMap["Print version"] = FreetureMode::PRINT_VERSION;
    enumMap["Continuous acquisition"] = FreetureMode::CONTINUOUS_ACQUISITION;
    enumMap["Meteor detection"] = FreetureMode::METEOR_DETECTION;
    enumMap["Single acquisition"] = FreetureMode::SINGLE_ACQUISITION;
    enumMap["Clean logs"] = FreetureMode::CLEAN_LOGS;
    enumMap["List devices"] = FreetureMode::LIST_DEVICES;
    enumMap["List available formats"] = FreetureMode::LIST_FORMATS;
    enumMap["Unknown mode"] = FreetureMode::UNKNOWN;
}
