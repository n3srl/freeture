/*
                            CfgLoader.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                               GEOPS-UPSUD-CNRS
*                       2019-2024 Andrea Novati
*                               N3 S.r.l.
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
*   Last modified:      14/02/2024
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    CfgLoader.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    13/06/2014
* \brief   Methods to fetch parameters from configuration file.
*/

#include "CfgLoader.h"

#include "Logger.h"

#include "CommonConfig.h"

#include <string>

#include <boost/program_options.hpp>
#include <boost/any.hpp>

namespace po = boost::program_options;

using namespace freeture;
using namespace std;

CfgLoader::CfgLoader(void){}

/// <summary>
/// Clear internal file configuration
/// </summary>
void CfgLoader::Clear()
{
    mData.clear();
}

/// <summary>
/// Load freeture configuration from file into a map
/// </summary>
/// <param name="file"></param>
/// <returns></returns>
bool CfgLoader::Load(const std::string& file) {
    try {
        LOG_INFO << "Loading configuration file " << file << "..." << endl;
        std::ifstream in_file(file.c_str());

        if (!in_file.good()) {
            LOG_ERROR << "CfgLoader::Load; file not found " << file << endl;
            return false;
        }

        po::options_description settings("Settings");

        settings.add_options()
            ("CAMERA_ID", po::value<std::string>()->default_value(DEFAULT_CAMERA_ID), "Identification number of the camera to use. Use \"freeture -l\" command to list available devices.")
            ("CAMERA_SERIAL", po::value<std::string>()->default_value(DEFAULT_CAMERA_SERIAL), "")
            ("CAMERA_INIT", po::value<std::string>()->default_value(DEFAULT_CAMERA_INIT), "")
            ("CAMERA_INIT_CONFIG", po::value<std::string>()->default_value(DEFAULT_CAMERA_INIT_CONFIG), "")
            ("INPUT_VIDEO_PATH", po::value<std::string>()->default_value(DEFAULT_INPUT_VIDEO_PATH), "List of videos to analyse. Use \",\" separator between paths.")
            ("INPUT_FRAMES_DIRECTORY_PATH", po::value<std::string>()->default_value(DEFAULT_INPUT_FRAMES_DIRECTORY_PATH), "List of frames directory (in fits). Use \",\" separator between paths.")
            ("INPUT_TIME_INTERVAL", po::value<std::string>()->default_value(DEFAULT_INPUT_TIME_INTERVAL), "Time (in ms) between two fits loading or two video frames. Used if the input is a video or a frames directory to give more time for the detection process to analyse frames.")
            ("ACQ_FPS", po::value<std::string>()->default_value(DEFAULT_ACQ_FPS), "Camera's acquisition frequency.")
            ("ACQ_FORMAT", po::value<std::string>()->default_value(DEFAULT_ACQ_FORMAT), "Camera's acquisition format. Use \"--listformats\" option with a \"-d deviceIndex\" to see available pixel formats.")
            ("ACQ_RES_CUSTOM_SIZE", po::value<std::string>()->default_value(DEFAULT_ACQ_RES_CUSTOM_SIZE), "Use custom camera resolution.")
            ("ACQ_OFFSET", po::value<std::string>()->default_value(DEFAULT_ACQ_OFFSET), "Specify camera x,y offset. Format: (x),(y).")
            ("ACQ_RES_SIZE", po::value<std::string>()->default_value(DEFAULT_ACQ_RES_SIZE), "Specify camera resolution. Format: (width)x(height).")
            ("SHIFT_BITS", po::value<std::string>()->default_value(DEFAULT_SHIFT_BITS), "DMK Gige Cameras use last 12 bits in MONO12. This parameter shifts bits in order to use the first 12 bits.")
            ("ACQ_AUTOEXPOSURE_ENABLED", po::value<std::string>()->default_value(DEFAULT_ACQ_AUTOEXPOSURE_ENABLED), "Enable auto-exposure controls on sunrise and sunset.")
            ("ACQ_MASK_ENABLED", po::value<std::string>()->default_value(DEFAULT_ACQ_MASK_ENABLED), "Enable to use a mask.")
            ("ACQ_MASK_PATH", po::value<std::string>()->default_value(DEFAULT_ACQ_MASK_PATH), "Location of the mask.")
            ("ACQ_BUFFER_SIZE", po::value<std::string>()->default_value(DEFAULT_ACQ_BUFFER_SIZE), "Size of the frame buffer (in seconds).")
            ("ACQ_NIGHT_EXPOSURE", po::value<std::string>()->default_value(DEFAULT_ACQ_NIGHT_EXPOSURE), "Fix exposure time during the night (us).")
            ("ACQ_NIGHT_GAIN", po::value<std::string>()->default_value(DEFAULT_ACQ_NIGHT_GAIN), "Fix gain during the night.")
            ("ACQ_DAY_EXPOSURE", po::value<std::string>()->default_value(DEFAULT_ACQ_DAY_EXPOSURE), "Fix exposure time during daytime (us). (Applied from end of sunrise until start of sunset)")
            ("ACQ_DAY_GAIN", po::value<std::string>()->default_value(DEFAULT_ACQ_DAY_GAIN), "Fix gain during daytime. (Applied from end of sunrise until start of sunset)")
            ("EPHEMERIS_ENABLED", po::value<std::string>()->default_value(DEFAULT_EPHEMERIS_ENABLED), "Enable auto computation of sun ephemeris.")
            ("SUN_HORIZON_1", po::value<std::string>()->default_value(DEFAULT_SUN_HORIZON_1), "Sun horizon height (in degree) where it's the start of sunrise and the end of sunset.")
            ("SUN_HORIZON_2", po::value<std::string>()->default_value(DEFAULT_SUN_HORIZON_2), "Sun horizon height (in degree) where it's the end of sunrise and the start of sunset.")
            ("SUNRISE_TIME", po::value<std::string>()->default_value(DEFAULT_SUNRISE_TIME), "Start of sunrise (UT) if EPHEMERIS_ENABLED is disabled.")
            ("SUNSET_TIME", po::value<std::string>()->default_value(DEFAULT_SUNSET_TIME), "Start of sunset (UT) if EPHEMERIS_ENABLED is disabled.")
            ("SUNSET_DURATION", po::value<std::string>()->default_value(DEFAULT_SUNSET_DURATION), "Duration of the sunset if EPHEMERIS_ENABLED is disabled.")
            ("SUNRISE_DURATION", po::value<std::string>()->default_value(DEFAULT_SUNRISE_DURATION), "Duration of the sunrise if EPHEMERIS_ENABLED is disabled.")
            ("EXPOSURE_CONTROL_ENABLED", po::value<std::string>()->default_value(DEFAULT_EXPOSURE_CONTROL_ENABLED), "enable/disable exposure control")
            ("EXPOSURE_CONTROL_FREQUENCY", po::value<std::string>()->default_value(DEFAULT_EXPOSURE_CONTROL_FREQUENCY), "Time interval (seconds) between two exposure control.")
            ("EXPOSURE_CONTROL_SAVE_IMAGE", po::value<std::string>()->default_value(DEFAULT_EXPOSURE_CONTROL_SAVE_IMAGE), "Enable to save final image with auto exposure control.")
            ("EXPOSURE_CONTROL_SAVE_INFOS", po::value<std::string>()->default_value(DEFAULT_EXPOSURE_CONTROL_SAVE_INFOS), "Enable to save informations in a .txt file about auto exposure control process.")
            ("ACQ_REGULAR_ENABLED", po::value<std::string>()->default_value(DEFAULT_ACQ_REGULAR_ENABLED), "Enable regular single capture. If enabled, ACQ_SCHEDULE_ENABLED has to be disabled.")
            ("ACQ_REGULAR_MODE", po::value<std::string>()->default_value(DEFAULT_ACQ_REGULAR_MODE), "Possible values: [DAY] | [NIGHT] | [DAYNIGHT]: [DAY] (Images are only regularly captured from start of dawn until end of twilight) [NIGHT] (Images are only regularly captured from end of twilight until start of dawn ) [DAYNIGHT] (Images are always regularly captured)")
            ("ACQ_REGULAR_CFG", po::value<std::string>()->default_value(DEFAULT_ACQ_REGULAR_CFG), "Specify the time interval of captures, exposure time, gain, format.")
            ("ACQ_REGULAR_PRFX", po::value<std::string>()->default_value(DEFAULT_ACQ_REGULAR_PRFX), "Captured image prefix.")
            ("ACQ_REGULAR_OUTPUT", po::value<std::string>()->default_value(DEFAULT_ACQ_REGULAR_OUTPUT), "Captured image format. [JPEG] | [FITS]")
            ("ACQ_SCHEDULE_ENABLED", po::value<std::string>()->default_value(DEFAULT_ACQ_SCHEDULE_ENABLED), "Enable scheduled acquisition. If enabled, ACQ_REGULAR_ENABLED has to be disabled.")
            ("ACQ_SCHEDULE", po::value<std::string>()->default_value(DEFAULT_ACQ_SCHEDULE), "Schedule (UT). Format is: .h.m.s.e.g.f.n where \".\" is a number) (e = exposure, g = gain, f = format index, n =repetition)")
            ("ACQ_SCHEDULE_OUTPUT", po::value<std::string>()->default_value(DEFAULT_ACQ_SCHEDULE_OUTPUT), "Captured image format. [JPEG] | [FITS]")
            ("DET_ENABLED", po::value<std::string>()->default_value(DEFAULT_DET_ENABLED), "Enable detection process.")
            ("DET_MODE", po::value<std::string>()->default_value(DEFAULT_DET_MODE), "Detection mode. [DAY] | [NIGHT] | [DAYNIGHT]")
            ("DET_DEBUG", po::value<std::string>()->default_value(DEFAULT_DET_DEBUG), "Enable debug of the detection process. (used for fits frames or video in input)")
            ("DET_DEBUG_PATH", po::value<std::string>()->default_value(DEFAULT_DET_DEBUG_PATH), "Location of debug data.")
            ("DET_TIME_AROUND", po::value<std::string>()->default_value(DEFAULT_DET_TIME_AROUND), "Time to keep before and after an event (seconds).")
            ("DET_TIME_MAX", po::value<std::string>()->default_value(DEFAULT_DET_TIME_MAX), "Maximum duration of an event. Available range: 1 to 30 seconds.")
            ("DET_METHOD", po::value<std::string>()->default_value(DEFAULT_DET_METHOD), "Choose a detection method.")
            ("DET_SAVE_FITS3D", po::value<std::string>()->default_value(DEFAULT_DET_SAVE_FITS3D), "Save fits3D in output.")
            ("DET_SAVE_FITS2D", po::value<std::string>()->default_value(DEFAULT_DET_SAVE_FITS2D), "Save fits2D in output.")
            ("DET_SAVE_SUM", po::value<std::string>()->default_value(DEFAULT_DET_SAVE_SUM), "Stack the event's frames.")
            ("DET_SUM_REDUCTION", po::value<std::string>()->default_value(DEFAULT_DET_SUM_REDUCTION), "Reduce sum to 16 bits file.")
            ("DET_SUM_MTHD", po::value<std::string>()->default_value(DEFAULT_DET_SUM_MTHD), "Sum reduction method.")
            ("DET_SAVE_SUM_WITH_HIST_EQUALIZATION", po::value<std::string>()->default_value(DEFAULT_DET_SAVE_SUM_WITH_HIST_EQUALIZATION), "Enable histogram equalization on DET_SUM_MTHD")
            ("DET_SAVE_AVI", po::value<std::string>()->default_value(DEFAULT_DET_SAVE_AVI), "Save a film .avi in output.")
            ("DET_UPDATE_MASK", po::value<std::string>()->default_value(DEFAULT_DET_UPDATE_MASK), "Enable auto-masking.")
            ("DET_UPDATE_MASK_FREQUENCY", po::value<std::string>()->default_value(DEFAULT_DET_UPDATE_MASK_FREQUENCY), "Frequency of auto-masking evaluation. (In seconds)")
            ("DET_DEBUG_UPDATE_MASK", po::value<std::string>()->default_value(DEFAULT_DET_DEBUG_UPDATE_MASK), "Enable to debug auto-masking process.")
            ("DET_DOWNSAMPLE_ENABLED", po::value<std::string>()->default_value(DEFAULT_DET_DOWNSAMPLE_ENABLED), "Enable to downsample (/2) frames. Can be set to false for resolution under 640x480. To true otherwise.")
            ("DET_SAVE_GEMAP", po::value<std::string>()->default_value(DEFAULT_DET_SAVE_GEMAP), "Save map of the global event.")
            ("DET_SAVE_DIRMAP", po::value<std::string>()->default_value(DEFAULT_DET_SAVE_DIRMAP), "Save direction map of an event.")
            ("DET_SAVE_POS", po::value<std::string>()->default_value(DEFAULT_DET_SAVE_POS), "Save in a .txt file the approximate position x,y of the event.")
            ("DET_LE_MAX", po::value<std::string>()->default_value(DEFAULT_DET_LE_MAX), "Maximum local event to extract on a single frame.")
            ("DET_GE_MAX", po::value<std::string>()->default_value(DEFAULT_DET_GE_MAX), "Maximum number of global event to track over time.")
            ("STACK_ENABLED", po::value<std::string>()->default_value(DEFAULT_STACK_ENABLED), "Enable to stack frames.")
            ("STACK_MODE", po::value<std::string>()->default_value(DEFAULT_STACK_MODE), "Stack mode. [DAY] | [NIGHT] | [DAYNIGHT]")
            ("STACK_TIME", po::value<std::string>()->default_value(DEFAULT_STACK_TIME), "Integration time of the stack (seconds).")
            ("STACK_INTERVAL", po::value<std::string>()->default_value(DEFAULT_STACK_INTERVAL), "Time to wait before to start a new stack.")
            ("STACK_MTHD", po::value<std::string>()->default_value(DEFAULT_STACK_MTHD), "Stack method. [MEAN] | [SUM]")
            ("STACK_REDUCTION", po::value<std::string>()->default_value(DEFAULT_STACK_REDUCTION), "Allowed dynamic reduction (Save in 16 bits instead of 32 bits)")
            ("DATA_PATH", po::value<std::string>()->default_value(DEFAULT_DATA_PATH), "Path where to save data.")
            ("LOG_PATH", po::value<std::string>()->default_value(DEFAULT_LOG_PATH), "Path of logs files.")
            ("LOG_ARCHIVE_DAY", po::value<std::string>()->default_value(DEFAULT_LOG_ARCHIVE_DAY), "Time to keep archive.")
            ("LOG_SIZE_LIMIT", po::value<std::string>()->default_value(DEFAULT_LOG_SIZE_LIMIT), "Limit size of logs on the hard disk (mo)")
            ("LOG_SEVERITY", po::value<std::string>()->default_value(DEFAULT_LOG_SEVERITY), "Level of messages to save in log files. [normal] | [notification] | [fail] | [warning] | [critical]")
            ("FITS_COMPRESSION", po::value<std::string>()->default_value(DEFAULT_FITS_COMPRESSION), "Enable compression (Experimental. Original fitskeys will be lost)")
            ("FITS_COMPRESSION_METHOD", po::value<std::string>()->default_value(DEFAULT_FITS_COMPRESSION_METHOD), "Specify cfitsio compression method by enclosing its parameters in square brackets.")
            ("STATION_NAME", po::value<std::string>()->default_value(DEFAULT_STATION_NAME), "Name of the station.")
            ("TELESCOP", po::value<std::string>()->default_value(DEFAULT_TELESCOP), "Station name.")
            ("OBSERVER", po::value<std::string>()->default_value(DEFAULT_OBSERVER), "Person in charge.")
            ("INSTRUMENT", po::value<std::string>()->default_value(DEFAULT_INSTRUMENT), "Instrument name.")
            ("CAMERA", po::value<std::string>()->default_value(DEFAULT_CAMERA), "Camera model name.")
            ("FOCAL", po::value<std::string>()->default_value(DEFAULT_FOCAL), "Camera focal.")
            ("APERTURE", po::value<std::string>()->default_value(DEFAULT_APERTURE), "Camera aperture.")
            ("SITELONG", po::value<std::string>()->default_value(DEFAULT_SITELONG), "Longitude observatory.")
            ("SITELAT", po::value<std::string>()->default_value(DEFAULT_SITELAT), "Latitude observatory.")
            ("SITEELEV", po::value<std::string>()->default_value(DEFAULT_SITEELEV), "Elevation observatory.")
            ("FILTER", po::value<std::string>()->default_value(DEFAULT_FILTER), "R = K1 * f * sin(theta/K2)")
            ("K1", po::value<std::string>()->default_value(DEFAULT_K1), "R = K1 * f * sin(theta/K2)")
            ("K2", po::value<std::string>()->default_value(DEFAULT_K2), "R = K1 * f * sin(theta/K2)")
            ("COMMENT", po::value<std::string>()->default_value(DEFAULT_COMMENT), "made possible by Andrea Novati (andrea.novati@gmail.com) and freeture " VERSION)
            ("CD1_1", po::value<std::string>()->default_value(DEFAULT_CD1_1), "deg/pix")
            ("CD1_2", po::value<std::string>()->default_value(DEFAULT_CD1_2), "deg/pix")
            ("CD2_1", po::value<std::string>()->default_value(DEFAULT_CD2_1), "deg/pix")
            ("CD2_2", po::value<std::string>()->default_value(DEFAULT_CD2_2), "deg/pix")
            ("XPIXEL", po::value<std::string>()->default_value(DEFAULT_XPIXEL), "physical's size of a pixel in micro meter")
            ("YPIXEL", po::value<std::string>()->default_value(DEFAULT_YPIXEL), "physical's size of a pixel in micro meter")
            ("MAIL_DETECTION_ENABLED", po::value<std::string>()->default_value(DEFAULT_MAIL_DETECTION_ENABLED), "Allow mail notifications.")
            ("MAIL_SMTP_SERVER", po::value<std::string>()->default_value(DEFAULT_MAIL_SMTP_SERVER), "SMTP server to send mail.")
            ("MAIL_CONNECTION_TYPE", po::value<std::string>()->default_value(DEFAULT_MAIL_CONNECTION_TYPE), "Enable or disable SMTP server authentication.")
            ("MAIL_SMTP_LOGIN", po::value<std::string>()->default_value(DEFAULT_MAIL_SMTP_LOGIN), "SMTP server user.")
            ("MAIL_SMTP_PASSWORD", po::value<std::string>()->default_value(DEFAULT_MAIL_SMTP_PASSWORD), "Password encoded in base64.")
            ("MAIL_RECIPIENT", po::value<std::string>()->default_value(DEFAULT_MAIL_RECIPIENT), "Recipients of mail notifications. Use \",\" as separators between mail addresses.")
            ;

        po::variables_map vm;

        po::store(po::parse_config_file(in_file, settings, true), vm);
        notify(vm);

        for (const auto& it : vm) {
            string value = vm[it.first].as<std::string>();
            string key = it.first;
            // Assuming all values can be straightforwardly converted to strings
            // You might need to handle different types specifically if necessary
            //mData[it.first] = value.type() == typeid(std::string) ? vm[it.first].as<std::string>() : std::to_string(vm[it.first].as<int>());
            mData[key] = value;
            LOG_DEBUG << "CfgLoader::Load;" << key << "=" << value << endl;

        }
    }
    catch (exception& e) {
        LOG_DEBUG << "CfgLoader::Load;" << e.what() << endl;
        LOG_ERROR << "KO" << endl;
        return false;
    }

    LOG_INFO << "OK" << endl;

    return true;
}

/// <summary>
/// Check if map contains key
/// </summary>
/// <param name="key"></param>
/// <returns></returns>
bool CfgLoader::Contains(const std::string& key) const{
    return mData.find(key) != mData.end();
}

/// <summary>
/// Get value for the selected key and returns true if exists
/// </summary>
/// <param name="key"></param>
/// <param name="value"></param>
/// <returns></returns>
bool CfgLoader::Get(const std::string& key, std::string& value) const{

    std::map<std::string,std::string>::const_iterator iter = mData.find(key);

    if(iter != mData.end()){

        value = iter->second;
        return true;
    } else {

        return false;
    }
}

/// <summary>
/// Get value as int for the selected key and returns true if exists
/// </summary>
/// <param name="key"></param>
/// <param name="value"></param>
/// <returns></returns>
bool CfgLoader::Get(const std::string& key, int& value) const{

    std::string str;

    if(Get(key, str)){

        value = atoi(str.c_str());
        return true;

    } else {

        return false;
    }
}

/// <summary>
/// Get value as long for the selected key and returns true if exists
/// </summary>
/// <param name="key"></param>
/// <param name="value"></param>
/// <returns></returns>
bool CfgLoader::Get(const std::string& key, long& value) const{

    std::string str;

    if(Get(key, str)){

        value = atol(str.c_str());
        return true;

    }else{

        return false;
    }
}

/// <summary>
/// Get value as double for the selected key and returns true if exists
/// </summary>
/// <param name="key"></param>
/// <param name="value"></param>
/// <returns></returns>
bool CfgLoader::Get(const std::string& key, double& value) const{

    std::string str;

    if(Get(key, str)){

        value = atof(str.c_str());
        return true;

    }else{

        return false;
    }
}

/// <summary>
/// Get value as bool for the selected key and returns true if exists
/// </summary>
/// <param name="key"></param>
/// <param name="value"></param>
/// <returns></returns>
bool CfgLoader::Get(const std::string& key, bool& value) const{

    std::string str;

    if(Get(key, str)){

        value = (str == "true");
        return true;

    }else{

        return false;
    }
}