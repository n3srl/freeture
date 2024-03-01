/*
                            CfgParam.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2016 Yoan Audureau, Chiara Marmo
*                       2017-2018 Chiara Marmo
*                                        GEOPS-UPSUD-CNRS
*                       2019-2024 Andrea Novati
*                                        N3 S.r.l.
* 
*                               
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
*   Last modified:      20/03/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include "CfgParam.h"
#include "SParam.h"

#include "CameraFirstInit.h"
#include "Logger.h"
#include "EParser.h"
#include "Conversion.h"
#include "Fits2D.h"


using namespace std;
using namespace freeture;

namespace fs = boost::filesystem;

/// <summary>
/// Load configuration file into memory mapped configuration
/// </summary>
/// <param name="cfgFilePath"></param>
CfgParam::CfgParam( string cfgFilePath)
{
    m_CfgFilePath = cfgFilePath;
    // Initialize parameters.

    enableErrors = false;

    pair<int,bool> var1(-1, false);
    pair<pair<int,bool>,string> var2(var1, "");
    m_Param.DEVICE_ID = -1;
    m_Param.data.status = false;
    m_Param.camInput.status = false;
    m_Param.det.status = false;
    m_Param.fitskeys.status = false;
    m_Param.framesInput.status = false;
    m_Param.log.status = false;
    m_Param.vidInput.status = false;
    m_Param.st.status = false;
    m_Param.station.status = false;
    m_Param.mail.status = false;

    m_Param.data.DATA_PATH = "./";
    m_Param.data.FITS_COMPRESSION = false;
    m_Param.data.FITS_COMPRESSION_METHOD = "[compress]";

    m_Param.log.LOG_ARCHIVE_DAY = 5;
    m_Param.log.LOG_PATH = "./";
    m_Param.log.LOG_SEVERITY = LogSeverityLevel::notification;
    m_Param.log.LOG_SIZE_LIMIT = 50;

    vector<string> finput, vinput;
    m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH = finput;
    m_Param.vidInput.INPUT_VIDEO_PATH = vinput;
    m_Param.framesInput.INPUT_TIME_INTERVAL = 0;
    m_Param.vidInput.INPUT_TIME_INTERVAL = 0;

    m_Param.camInput.ACQ_DAY_EXPOSURE = 0;
    m_Param.camInput.ACQ_DAY_GAIN = 0;
    m_Param.camInput.ACQ_FORMAT = CamPixFmt::MONO8;
    m_Param.camInput.ACQ_FPS = 30;
    m_Param.camInput.ACQ_HEIGHT = 1080;
    m_Param.camInput.ACQ_NIGHT_EXPOSURE = 0;
    m_Param.camInput.ACQ_NIGHT_GAIN = 0;
    m_Param.camInput.ACQ_RES_CUSTOM_SIZE = false;
    m_Param.camInput.ACQ_STARTX = 0;
    m_Param.camInput.ACQ_STARTY = 0;
    m_Param.camInput.ACQ_WIDTH = 1440;
    m_Param.camInput.ephem.EPHEMERIS_ENABLED = false;
    m_Param.camInput.ephem.SUNRISE_DURATION = 3600;
    vector<int>sunrisetime, sunsettime;

    sunrisetime.push_back(7);
    sunrisetime.push_back(0);
    sunsettime.push_back(22);
    sunsettime.push_back(0);

    m_Param.camInput.ephem.SUNRISE_TIME = sunrisetime;
    m_Param.camInput.ephem.SUNSET_DURATION = 3600;
    m_Param.camInput.ephem.SUNSET_TIME = sunsettime;
    m_Param.camInput.EXPOSURE_CONTROL_FREQUENCY = 300;
    m_Param.camInput.EXPOSURE_CONTROL_SAVE_IMAGE = false;
    m_Param.camInput.EXPOSURE_CONTROL_SAVE_INFOS = false;
    m_Param.camInput.regcap.ACQ_REGULAR_ENABLED = false;
    m_Param.camInput.schcap.ACQ_SCHEDULE_ENABLED = false;
    m_Param.camInput.SHIFT_BITS = false;

    m_Param.det.ACQ_BUFFER_SIZE = 15;
    m_Param.det.ACQ_MASK_ENABLED = false;
    m_Param.det.ACQ_AUTOEXPOSURE_ENABLED = false;
    m_Param.det.DET_DEBUG = false;
    m_Param.det.DET_DEBUG_UPDATE_MASK = false;
    m_Param.det.DET_DOWNSAMPLE_ENABLED = true;
    m_Param.det.DET_ENABLED = false;

    m_Param.st.STACK_ENABLED = false;

    m_Param.mail.MAIL_DETECTION_ENABLED = false;

    m_Param.station.STATION_NAME = "STATION";
    m_Param.station.SITEELEV = 0.0;
    m_Param.station.SITELAT = 0.0;
    m_Param.station.SITELONG = 0.0;

    // Load parameters.

    fs::path pcfg(cfgFilePath);
    if(fs::exists(pcfg)) {
        if(m_Cfg.Load(cfgFilePath)) {
            loadDeviceID();
            loadDataParam();
            loadLogParam();
            loadDetParam();
            loadStackParam();
            loadStationParam();
            loadFitskeysParam();
            loadMailParam();
            loadVidParam();
            loadFramesParam();
            loadCamParam();

        } else {
            m_EMsg.push_back("Fail to load configuration file.");
            LOG_ERROR << "Fail to load configuration file." << endl;
        }
    }else{
        m_EMsg.push_back("Configuration file path not exists : " + cfgFilePath);
        LOG_ERROR << "Configuration file path not exists : " << cfgFilePath << endl;
    }
}

void CfgParam::setInitRequired(bool init)
{
    cout << "Set init req" << endl;
    FILE* ofile = fopen(FREETURE_WRITE_CONFIG_PATH, "w");
    std::ifstream file(m_CfgFilePath);
    if(!file.is_open())
    {
        LOG_ERROR << "reset configuration file failed." << endl;
        return;
    }

    string line;
    while (!file.eof())
    {
        getline(file, line);
        size_t found = line.find("CAMERA_INIT = true");
        if(found != string::npos)
        {
            line = "CAMERA_INIT = false";
        }
        //cout << "Write line " << line << endl;
        line += '\n';
        fprintf(ofile, "%s", line.c_str());
    }

    fclose(ofile);
    file.close();
    const char* newFileName = (m_CfgFilePath  + ".old").c_str();
    rename(m_CfgFilePath.c_str(), newFileName);
    rename(FREETURE_WRITE_CONFIG_PATH, m_CfgFilePath.c_str());
    remove(newFileName);
    return;
}

bool CfgParam::loadCameraInit()
{
    bool error = false;

    string defaultCameraConfigFile = "/freeture/cinit.cfg";
    bool cBool = false;
    if(!m_Cfg.Get("CAMERA_INIT", cBool))
    {
        m_Param.CAMERA_INIT = false;
    } else 
    {
        m_Param.CAMERA_INIT = cBool;
    } 

    string cString = "";
    if(!m_Cfg.Get("CAMERA_INIT_CONFIG", cString))
    {
        m_Param.CAMERA_INIT_CONFIG = defaultCameraConfigFile;
    } 
    else {
        m_Param.CAMERA_INIT_CONFIG = cString;
    }
    
    if(m_Param.CAMERA_INIT_CONFIG == "") m_Param.CAMERA_INIT_CONFIG = defaultCameraConfigFile;

    string message;
    if(m_Param.CAMERA_INIT)
    {

        if(FILE* file = fopen(FREETURE_CHECK_INIT_DONE, "r")) {
            fclose(file);
            error = true;
        } else 
        {
            m_EMsg.push_back("Error loading " FREETURE_CHECK_INIT_DONE);
            LOG_ERROR << "CfgParam::loadCameraInit" << "Error loading " << FREETURE_CHECK_INIT_DONE << endl;
            error = false;
        }

        if(error)
        {
            setInitRequired(true);
            remove(FREETURE_CHECK_INIT_DONE);
        }

        message = "YES";
        LOG_INFO   << "Camera init file " << m_Param.CAMERA_INIT_CONFIG << endl;
    } else 
    {
        message = "NO";
    }


    LOG_INFO << "Need to init camera: " << message << endl;
    return error;
}

void CfgParam::loadCameraSerial()
{
    string cString;

    bool failStringSerial = false;
    string failmsg = " CAMERA_SERIAL : ";

    if(!m_Cfg.Get("CAMERA_SERIAL", cString))
    {
        failStringSerial = true;
        failmsg += "Fail to get camera serial, DeviceId will be used for camera selection";
    }

    m_Param.CAMERA_SERIAL = cString;
}

void CfgParam::loadDeviceID() {

    int cId;
    string cString;
    bool failIntId, failStringId = false;
    string failmsg = " CAMERA_ID : ";

    if(!m_Cfg.Get("CAMERA_ID", cId)) {
        failIntId = true;
        failmsg += "Fail to get value. Probably not defined.\n";
    }

    else if(!m_Cfg.Get("CAMERA_ID", cString)) {
        failStringId = true;
        failmsg += "Fail to get value. Probably not defined.\n";
    }

    m_Param.DEVICE_ID = cId;
}

void CfgParam::loadDataParam() {

    bool e = false;

    if(!m_Cfg.Get("DATA_PATH", m_Param.data.DATA_PATH)) {
        m_Param.data.errormsg.push_back("DATA_PATH : Fail to get value.");
        e = true;
    }else{

        fs::path p(m_Param.data.DATA_PATH);

        if(!fs::exists(p)){
            try {
                fs::create_directory(p);
            } catch (exception &ex) {
                m_Param.data.errormsg.push_back("DATA_PATH : Can't create Data Path directory: " + m_Param.data.DATA_PATH);
                e = true;
            }
        }
    }

    if(!m_Cfg.Get("FITS_COMPRESSION", m_Param.data.FITS_COMPRESSION)) {
        m_Param.data.errormsg.push_back(" FITS_COMPRESSION : Fail to get value.");
        e = true;
    }else{

        m_Param.data.FITS_COMPRESSION_METHOD = "";

        if(m_Param.data.FITS_COMPRESSION) {
            if(!m_Cfg.Get("FITS_COMPRESSION_METHOD", m_Param.data.FITS_COMPRESSION_METHOD)) {
                m_Param.data.errormsg.push_back(" FITS_COMPRESSION_METHOD : Fail to get value.");
                e = true;
            }
        }
    }

    if(!e) m_Param.data.status = true;
}

void CfgParam::loadLogParam() {

    bool e = false;

    if(!m_Cfg.Get("LOG_PATH", m_Param.log.LOG_PATH)) {
        m_Param.log.errormsg.push_back(" LOG_PATH : Fail to get value.");
        e = true;
    }else{

        fs::path p(m_Param.log.LOG_PATH);

        if(!fs::exists(p)){
            try {
                fs::create_directory(p);
            } catch (exception &ex) {
                m_Param.log.errormsg.push_back(" LOG_PATH : Can't create Log Path directory.");
                e = true;
            }
        }
    }

    if(!m_Cfg.Get("LOG_ARCHIVE_DAY", m_Param.log.LOG_ARCHIVE_DAY)) {
        m_Param.log.errormsg.push_back(" LOG_ARCHIVE_DAY : Fail to get value.");
        e = true;
    }

    if(!m_Cfg.Get("LOG_SIZE_LIMIT", m_Param.log.LOG_SIZE_LIMIT)) {
        m_Param.log.errormsg.push_back(" LOG_SIZE_LIMIT : Fail to get value.");
        e = true;
    }

    string log_severity;
    EParser<LogSeverityLevel> log_sev;
    if(!m_Cfg.Get("LOG_SEVERITY", log_severity)) {
        m_Param.log.errormsg.push_back(" LOG_SEVERITY : Fail to get value.");
        e = true;
    }

    try {
        m_Param.log.LOG_SEVERITY = log_sev.parseEnum("LOG_SEVERITY", log_severity);
    }catch (exception &ex) {
        m_Param.log.errormsg.push_back(" LOG_SEVERITY : " + string(ex.what()));
        e = true;
    }

    if(!e) m_Param.log.status = true;
}

void CfgParam::loadFramesParam() {

    bool e = false;

    if(!m_Cfg.Get("INPUT_TIME_INTERVAL", m_Param.framesInput.INPUT_TIME_INTERVAL)) {
        m_Param.framesInput.errormsg.push_back(" INPUT_TIME_INTERVAL : Fail to get value.");
        //cout << " INPUT_FRAMES_DIRECTORY_PATH : Fail to get value." << endl;
        e = true;
    }

    string inputPaths;
    if(!m_Cfg.Get("INPUT_FRAMES_DIRECTORY_PATH", inputPaths)) {
        m_Param.framesInput.errormsg.push_back(" INPUT_FRAMES_DIRECTORY_PATH : Fail to get value.");
        //cout << " INPUT_FRAMES_DIRECTORY_PATH : Fail to get value." << endl;
        e = true;
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(",");
    tokenizer tokens(inputPaths, sep);

    for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter){
        fs::path p_input_frames_dir(*tok_iter);
        if(!boost::filesystem::exists(p_input_frames_dir)) {
            m_Param.framesInput.errormsg.push_back(" INPUT_FRAMES_DIRECTORY_PATH : " + *tok_iter + " not exists.");
            e = true;
        }else{
            m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.push_back(*tok_iter);
        }
    }

    if(!e) m_Param.framesInput.status = true;
}

void CfgParam::loadVidParam() {

    bool e = false;

    if(!m_Cfg.Get("INPUT_TIME_INTERVAL", m_Param.vidInput.INPUT_TIME_INTERVAL)) {
        m_Param.vidInput.errormsg.push_back(" INPUT_TIME_INTERVAL : Fail to get value.");
        //cout << " INPUT_FRAMES_DIRECTORY_PATH : Fail to get value." << endl;
        e = true;
    }

    string input_video_path;
    if(!m_Cfg.Get("INPUT_VIDEO_PATH", input_video_path)) {
        m_Param.vidInput.errormsg.push_back(" INPUT_VIDEO_PATH : Fail to get value.");
        e = true;
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(",");
    tokenizer tokens(input_video_path, sep);

    for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter){
       fs::path p_input_video_path(*tok_iter);
        if(!is_regular_file(p_input_video_path)) {
            m_Param.vidInput.errormsg.push_back(" INPUT_VIDEO_PATH : " + *tok_iter + " not exists.");
            e = true;
        }else{
            m_Param.vidInput.INPUT_VIDEO_PATH.push_back(*tok_iter);
        }
    }

    if(!e) m_Param.vidInput.status = true;
}

void CfgParam::loadDetParam()
{

    bool error = false;

    if (!m_Cfg.Get("DET_ENABLED", m_Param.det.DET_ENABLED)) {
        error = true;
        m_Param.det.errormsg.push_back(" DET_ENABLED : Fail to load value.");
    }

    //if detections are enabled check other fields
    if (m_Param.det.DET_ENABLED)
    {

        if (!m_Cfg.Get("ACQ_BUFFER_SIZE", m_Param.det.ACQ_BUFFER_SIZE)) {
            error = true;
            m_Param.det.errormsg.push_back(" ACQ_BUFFER_SIZE : Fail to load value.");
        }
        
        if (!m_Cfg.Get("ACQ_AUTOEXPOSURE_ENABLED", m_Param.det.ACQ_AUTOEXPOSURE_ENABLED)) {
            m_Param.det.ACQ_AUTOEXPOSURE_ENABLED = false;
        }

        if (!m_Cfg.Get("ACQ_MASK_ENABLED", m_Param.det.ACQ_MASK_ENABLED)) {
            error = true;
            m_Param.det.errormsg.push_back(" ACQ_MASK_ENABLED : Fail to load value.");
        }
        else {

            if (m_Param.det.ACQ_MASK_ENABLED) {

                if (!m_Cfg.Get("ACQ_MASK_PATH", m_Param.det.ACQ_MASK_PATH)) {
                    error = true;
                    m_Param.det.errormsg.push_back(" ACQ_MASK_PATH : Fail to load value.");
                }
                else {
                    cv::Mat tempmask = cv::imread(m_Param.det.ACQ_MASK_PATH, cv::IMREAD_GRAYSCALE);

                    if (!tempmask.data) {
                        error = true;
                        m_Param.det.errormsg.push_back(" MASK : Fail to load the mask image. No data.");
                        // Add test to compare mask size to a capture from camera or video or frame file
                    }
                    else {
                        tempmask.copyTo(m_Param.det.MASK);
                    }
                }
            }
        }


        string det_mode;
        if (!m_Cfg.Get("DET_MODE", det_mode)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_MODE : Fail to load value.");
        }
        else {
            try {
                EParser<TimeMode> detMode;
                m_Param.det.DET_MODE = detMode.parseEnum("DET_MODE", det_mode);
            }
            catch (exception& ex) {
                error = true;
                m_Param.det.errormsg.push_back(" DET_MODE : " + string(ex.what()));
            }
        }

        if (!m_Cfg.Get("DET_DEBUG", m_Param.det.DET_DEBUG)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_DEBUG : Fail to load value.");
        }
        else {

            if (m_Param.det.DET_DEBUG) {

                if (!m_Cfg.Get("DET_DEBUG_PATH", m_Param.det.DET_DEBUG_PATH)) {
                    error = true;
                    m_Param.det.errormsg.push_back(" DET_DEBUG_PATH : Fail to load value.");
                }
                else {

                    fs::path p(m_Param.det.DET_DEBUG_PATH);

                    if (!fs::exists(p)) {
                        if (!fs::create_directory(p)) {
                            error = true;
                            m_Param.det.errormsg.push_back(" DET_DEBUG_PATH : Can't create Debug Path.");
                        }
                    }
                }
            }
        }

        if (!m_Cfg.Get("DET_TIME_AROUND", m_Param.det.DET_TIME_AROUND)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_TIME_AROUND : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_TIME_MAX", m_Param.det.DET_TIME_MAX)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_TIME_MAX : Fail to load value.");
        }

        string det_mthd;
        if (!m_Cfg.Get("DET_METHOD", det_mthd)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_METHOD : Fail to load value.");
        }
        else {
            try {
                EParser<DetMeth> detMthd;
                m_Param.det.DET_METHOD = detMthd.parseEnum("DET_METHOD", det_mthd);
            }
            catch (exception& ex) {
                error = true;
                m_Param.det.errormsg.push_back(" DET_METHOD : " + string(ex.what()));
            }
        }

        if (!m_Cfg.Get("DET_SAVE_FITS3D", m_Param.det.DET_SAVE_FITS3D)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SAVE_FITS3D : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_SAVE_FITS2D", m_Param.det.DET_SAVE_FITS2D)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SAVE_FITS2D : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_SAVE_SUM", m_Param.det.DET_SAVE_SUM)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SAVE_SUM : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_SUM_REDUCTION", m_Param.det.DET_SUM_REDUCTION)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SUM_REDUCTION : Fail to load value.");
        }

        string det_sum_mthd;
        if (!m_Cfg.Get("DET_SUM_MTHD", det_sum_mthd)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SUM_MTHD : Fail to load value.");
        }
        else {
            try {
                EParser<StackMeth> detSumMthd;
                m_Param.det.DET_SUM_MTHD = detSumMthd.parseEnum("DET_SUM_MTHD", det_sum_mthd);
            }
            catch (exception& ex) {
                error = true;
                m_Param.det.errormsg.push_back(" DET_SUM_MTHD : " + string(ex.what()));
            }
        }

        if (!m_Cfg.Get("DET_SAVE_SUM_WITH_HIST_EQUALIZATION", m_Param.det.DET_SAVE_SUM_WITH_HIST_EQUALIZATION)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SAVE_SUM_WITH_HIST_EQUALIZATION : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_SAVE_AVI", m_Param.det.DET_SAVE_AVI)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SAVE_AVI : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_UPDATE_MASK", m_Param.det.DET_UPDATE_MASK)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_UPDATE_MASK : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_UPDATE_MASK_FREQUENCY", m_Param.det.DET_UPDATE_MASK_FREQUENCY)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_UPDATE_MASK_FREQUENCY : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_DEBUG_UPDATE_MASK", m_Param.det.DET_DEBUG_UPDATE_MASK)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_DEBUG_UPDATE_MASK : Fail to load value.");
        }
        else {

            if (m_Param.det.DET_DEBUG_UPDATE_MASK) {

                if (!m_Cfg.Get("DET_DEBUG_PATH", m_Param.det.DET_DEBUG_PATH)) {
                    error = true;
                    m_Param.det.errormsg.push_back(" DET_DEBUG_PATH : Fail to load value.");
                }
                else {
                    fs::path p(m_Param.det.DET_DEBUG_PATH);

                    if (!fs::exists(p)) {
                        try {
                            fs::create_directory(p);
                        }
                        catch (exception& ex) {
                            m_Param.det.errormsg.push_back(" DET_DEBUG_PATH : Can't create Debug Path. Debug Path must exist as DET_DEBUG_UPDATE_MASK is enabled.");
                            error = true;
                        }
                    }
                }
            }
        }


        // --------------------------------------------------------------------------------------

        if (!m_Cfg.Get("DET_DOWNSAMPLE_ENABLED", m_Param.det.DET_DOWNSAMPLE_ENABLED)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_DOWNSAMPLE_ENABLED : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_SAVE_GEMAP", m_Param.det.temporal.DET_SAVE_GEMAP)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SAVE_GEMAP : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_SAVE_DIRMAP", m_Param.det.temporal.DET_SAVE_DIRMAP)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SAVE_DIRMAP : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_SAVE_POS", m_Param.det.temporal.DET_SAVE_POS)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_SAVE_POS : Fail to load value.");
        }

        if (!m_Cfg.Get("DET_LE_MAX", m_Param.det.temporal.DET_LE_MAX)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_LE_MAX : Fail to load value.");
        }
        else {

            if (m_Param.det.temporal.DET_LE_MAX < 1 || m_Param.det.temporal.DET_LE_MAX > 10) {

                error = true;
                m_Param.det.errormsg.push_back(" DET_LE_MAX : Available range is from 1 to 10.");

            }

        }

        if (!m_Cfg.Get("DET_GE_MAX", m_Param.det.temporal.DET_GE_MAX)) {
            error = true;
            m_Param.det.errormsg.push_back(" DET_GE_MAX : Fail to load value.");
        }
        else {

            if (m_Param.det.temporal.DET_GE_MAX < 1 || m_Param.det.temporal.DET_GE_MAX > 10) {

                error = true;
                m_Param.det.errormsg.push_back(" DET_GE_MAX : Available range is from 1 to 10.");

            }

        }
    }

    if (!error) m_Param.det.status = true;
}


void CfgParam::loadDetParam(InputDeviceType m_InputType)
{
    bool error = false;

    if (m_Param.det.DET_ENABLED)
    {

        // If input device type is frames or video, increase DET_TIME_MAX because
        // time can not be take account as the time interval between can be increased.
        if (m_InputType == VIDEO || m_InputType == SINGLE_FITS_FRAME) {
            m_Param.det.DET_TIME_MAX = 10000;
        }
        else {
            if (m_Param.det.DET_TIME_MAX <= 0 || m_Param.det.DET_TIME_MAX > 30) {
                error = true;
                m_Param.det.errormsg.push_back(" DET_TIME_MAX : Available range is from 1 to 30 seconds.");
            }
        }
    }
}

void CfgParam::loadStackParam()
{

    bool error = false;

    if (!m_Cfg.Get("STACK_ENABLED", m_Param.st.STACK_ENABLED)) {
        error = true;
        m_Param.st.errormsg.push_back(" STACK_ENABLED : Fail to load value.");
    }

    if (m_Param.st.STACK_ENABLED)
    {
        string stack_mode;
        if (!m_Cfg.Get("STACK_MODE", stack_mode)) {
            error = true;
            m_Param.st.errormsg.push_back(" STACK_MODE : Fail to load value.");
        }
        else {
            try {
                EParser<TimeMode> stackMode;
                m_Param.st.STACK_MODE = stackMode.parseEnum("STACK_MODE", stack_mode);
            }
            catch (exception& ex) {
                error = true;
                m_Param.st.errormsg.push_back(" STACK_MODE : " + string(ex.what()));
            }
        }

        if (!m_Cfg.Get("STACK_TIME", m_Param.st.STACK_TIME)) {
            error = true;
            m_Param.st.errormsg.push_back(" STACK_TIME : Fail to load value.");
        }

        if (!m_Cfg.Get("STACK_INTERVAL", m_Param.st.STACK_INTERVAL)) {
            error = true;
            m_Param.st.errormsg.push_back(" STACK_INTERVAL : Fail to load value.");
        }

        string stack_mthd;
        if (!m_Cfg.Get("STACK_MTHD", stack_mthd)) {
            error = true;
            m_Param.st.errormsg.push_back(" STACK_MTHD : Fail to load value.");
        }
        else {
            try {
                EParser<StackMeth> stackMthd;
                m_Param.st.STACK_MTHD = stackMthd.parseEnum("STACK_MTHD", stack_mthd);
            }
            catch (exception& ex) {
                error = true;
                m_Param.st.errormsg.push_back(" STACK_MTHD : " + string(ex.what()));
            }
        }

        if (!m_Cfg.Get("STACK_REDUCTION", m_Param.st.STACK_REDUCTION)) {
            error = true;
            m_Param.st.errormsg.push_back(" STACK_REDUCTION : Fail to load value.");
        }
    }

    if (!error) m_Param.st.status = true;
}

void CfgParam::loadStationParam() {

    bool e = false;

    if(!m_Cfg.Get("STATION_NAME", m_Param.station.STATION_NAME)) {
        e = true;
        m_Param.station.errormsg.push_back(" STATION_NAME : Fail to load value.");
    }

    if(!m_Cfg.Get("TELESCOP", m_Param.station.TELESCOP)) {
        e = true;
        m_Param.station.errormsg.push_back(" TELESCOP : Fail to load value.");
    }

    if(!m_Cfg.Get("OBSERVER", m_Param.station.OBSERVER)) {
        e = true;
        m_Param.station.errormsg.push_back(" OBSERVER : Fail to load value.");
    }

    if(!m_Cfg.Get("INSTRUMENT", m_Param.station.INSTRUMENT)) {
        e = true;
        m_Param.station.errormsg.push_back(" INSTRUME : Fail to load value.");
    }

    if(!m_Cfg.Get("CAMERA", m_Param.station.CAMERA)) {
        e = true;
        m_Param.station.errormsg.push_back(" CAMERA : Fail to load value.");
    }

    if(!m_Cfg.Get("FOCAL", m_Param.station.FOCAL)) {
        e = true;
        m_Param.station.errormsg.push_back(" FOCAL : Fail to load value.");
    }

    if(!m_Cfg.Get("APERTURE", m_Param.station.APERTURE)) {
        e = true;
        m_Param.station.errormsg.push_back(" APERTURE : Fail to load value.");
    }

    if(!m_Cfg.Get("SITELONG", m_Param.station.SITELONG)) {
        e = true;
        m_Param.station.errormsg.push_back(" SITELONG : Fail to load value.");
    }

    if(!m_Cfg.Get("SITELAT", m_Param.station.SITELAT)) {
        e = true;
        m_Param.station.errormsg.push_back(" SITELAT : Fail to load value.");
    }

    if(!m_Cfg.Get("SITEELEV", m_Param.station.SITEELEV)) {
        e = true;
        m_Param.station.errormsg.push_back(" SITEELEV : Fail to load value.");
    }

    if(!e) m_Param.station.status = true;
}

void CfgParam::loadFitskeysParam() {

    bool e = false;

    if(!m_Cfg.Get("K1", m_Param.fitskeys.K1)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" K1 : Fail to load value.");
    }

    if(!m_Cfg.Get("K2", m_Param.fitskeys.K2)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" K2 : Fail to load value.");
    }

    if(!m_Cfg.Get("FILTER", m_Param.fitskeys.FILTER)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" FILTER : Fail to load value.");
    }

    if(!m_Cfg.Get("CD1_1", m_Param.fitskeys.CD1_1)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" CD1_1 : Fail to load value.");
    }

    if(!m_Cfg.Get("CD1_2", m_Param.fitskeys.CD1_2)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" CD1_2 : Fail to load value.");
    }

    if(!m_Cfg.Get("CD2_1", m_Param.fitskeys.CD2_1)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" CD2_1 : Fail to load value.");
    }

    if(!m_Cfg.Get("CD2_2", m_Param.fitskeys.CD2_2)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" CD2_2 : Fail to load value.");
    }

    if(!m_Cfg.Get("XPIXEL", m_Param.fitskeys.XPIXEL)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" XPIXEL : Fail to load value.");
    }

    if(!m_Cfg.Get("YPIXEL", m_Param.fitskeys.YPIXEL)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" YPIXEL : Fail to load value.");
    }

    if(!m_Cfg.Get("COMMENT", m_Param.fitskeys.COMMENT)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back(" COMMENT : Fail to load value.");
    }

    if(!e) m_Param.fitskeys.status = true;
}

void CfgParam::loadMailParam() {

    bool e = false;

    if(!m_Cfg.Get("MAIL_DETECTION_ENABLED", m_Param.mail.MAIL_DETECTION_ENABLED)) {
        e = true;
        m_Param.mail.errormsg.push_back(" MAIL_DETECTION_ENABLED : Fail to load value.");
    }else{

        if(m_Param.mail.MAIL_DETECTION_ENABLED) {

            string mailRecipients;
            if(!m_Cfg.Get("MAIL_RECIPIENT", mailRecipients)) {
                e = true;
                m_Param.mail.errormsg.push_back(" MAIL_RECIPIENT : Fail to load value.");
            }else {

                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                boost::char_separator<char> sep(",");
                tokenizer tokens(mailRecipients, sep);

                for (tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter){
                    m_Param.mail.MAIL_RECIPIENTS.push_back(*tok_iter);
                }
            }

            if(!m_Cfg.Get("MAIL_SMTP_SERVER", m_Param.mail.MAIL_SMTP_SERVER)) {
                e = true;
                m_Param.mail.errormsg.push_back(" MAIL_SMTP_SERVER : Fail to load value.");
            }

            string smtp_connection_type;
            if(!m_Cfg.Get("MAIL_CONNECTION_TYPE", smtp_connection_type)) {
                e = true;
                m_Param.mail.errormsg.push_back(" MAIL_CONNECTION_TYPE : Fail to load value.");
            }else {
                try
                {
                    EParser<SmtpSecurity> smtp_security;
                    m_Param.mail.MAIL_CONNECTION_TYPE = smtp_security.parseEnum("MAIL_CONNECTION_TYPE", smtp_connection_type);

                    if(m_Param.mail.MAIL_CONNECTION_TYPE != NO_SECURITY) {

                        if(!m_Cfg.Get("MAIL_SMTP_LOGIN", m_Param.mail.MAIL_SMTP_LOGIN)) {
                            e = true;
                            m_Param.mail.errormsg.push_back(" MAIL_SMTP_LOGIN : Fail to load value.");
                        }

                        if(!m_Cfg.Get("MAIL_SMTP_PASSWORD", m_Param.mail.MAIL_SMTP_PASSWORD)) {
                            e = true;
                            m_Param.mail.errormsg.push_back(" MAIL_SMTP_PASSWORD : Fail to load value.");
                        }
                    }else{
                        m_Param.mail.MAIL_SMTP_LOGIN = "";
                        m_Param.mail.MAIL_SMTP_PASSWORD = "";
                    }

                }catch (exception &ex) {
                    e = true;
                    m_Param.mail.errormsg.push_back(" MAIL_CONNECTION_TYPE : " + string(ex.what()));
                }
            }
        }
    }

    if(!e) m_Param.mail.status = true;

}

dataParam CfgParam::getDataParam() {
    return m_Param.data;
}

logParam CfgParam::getLogParam() {
    return m_Param.log;
}

framesParam CfgParam::getFramesParam() {
    return m_Param.framesInput;
}

videoParam CfgParam::getVidParam() {
    return m_Param.vidInput;
}

cameraParam CfgParam::getCamParam() {
    return m_Param.camInput;
}

detectionParam CfgParam::getDetParam() {
    return m_Param.det;
}

stackParam CfgParam::getStackParam() {
    return m_Param.st;
}

stationParam CfgParam::getStationParam() {
    return m_Param.station;
}

fitskeysParam CfgParam::getFitskeysParam() {
    return m_Param.fitskeys;
}

mailParam CfgParam::getMailParam() {
    return m_Param.mail;
}

parameters CfgParam::getAllParam() {
    return m_Param;
}

bool CfgParam::checkDeviceID() {
    
    if(m_Param.DEVICE_ID == -1)
    {
        if(enableErrors) {
            LOG_ERROR << "CfgParam::checkDeviceID;" << "Device not valid" << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkDataParam() {
    if(!m_Param.data.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.data.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkDataParam;" << m_Param.data.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkLogParam() {
    if(!m_Param.log.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.log.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkLogParam;" << m_Param.log.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkFramesParam() {

    if(!m_Param.framesInput.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.framesInput.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkFramesParam;" << m_Param.framesInput.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkVidParam() {
    if(!m_Param.vidInput.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.vidInput.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkVidParam;" << m_Param.vidInput.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkCamParam() {
    if(!m_Param.camInput.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.camInput.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkCamParam;" << m_Param.camInput.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkDetParam() {
    if(!m_Param.det.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.det.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::detParamIsCorrect;" << m_Param.det.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkStackParam() {
    if(!m_Param.st.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.st.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkStackParam;" << m_Param.st.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkStationParam() {
    if(!m_Param.station.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.station.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkStationParam;" << m_Param.station.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkFitskeysParam() {
    if(!m_Param.fitskeys.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.fitskeys.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkFitskeysParam;" << m_Param.fitskeys.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::checkMailParam() {
    if(!m_Param.mail.status) {
        if(enableErrors) {
            for(int i = 0; i < m_Param.mail.errormsg.size(); i++)
                LOG_ERROR << "CfgParam::checkMailParam;" << m_Param.mail.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool CfgParam::allParamAreCorrect()
{
    bool errorFound = false;

    //check configuration file
   fs::path pcfg(m_CfgFilePath);

    if(!boost::filesystem::exists(pcfg))
    {
        errorFound = true;

        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Error. Configuration file not exists at " << m_CfgFilePath << " use -c to set a different location." << endl;
    }


    if(!checkDeviceID())
    {
        errorFound = true;
        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Errors on device ID. " << endl;
    }

    if(!checkDataParam()){
        errorFound = true;
        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Errors on data parameters. " << endl;
    }

    if(!checkLogParam()){
        errorFound = true;
        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Errors on log parameters. " << endl;
    }

    if(!checkDetParam()){
        errorFound = true;
        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Errors on detection parameters. " << endl;
    }

    if(!checkStackParam()){
        errorFound = true;
        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Errors on stack parameters. " << endl;
    }

    if(!checkStationParam()){
        errorFound = true;
        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Errors on station parameters. " << endl;
    }

    if(!checkFitskeysParam()){
        errorFound = true;
        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Errors on fitskeys parameters. " << endl;
    }

    if(!checkMailParam()){
        errorFound = true;
        if (enableErrors)
            LOG_ERROR << "CfgParam::allParamAreCorrect;" << ">> Errors on mail parameters. " << endl;
    }

    if(errorFound)
        return false;

    return true;
}

void CfgParam::loadCamParam()
{
    bool error = false;

    loadCameraSerial();

    if (!loadCameraInit()) 
    {
        m_Param.camInput.errormsg.push_back(" CAMERA_INIT : Missing parameter in configuration file");
        error = true;
    }
  
    if (!m_Cfg.Get("ACQ_FPS", m_Param.camInput.ACQ_FPS)) {
        m_Param.camInput.errormsg.push_back(" ACQ_FPS : Fail to get value.");
        error = true;
    }

    //-------------------------------------------------------------------

    string pixfmt;
    if (!m_Cfg.Get("ACQ_FORMAT", pixfmt)) {
        m_Param.camInput.errormsg.push_back(" ACQ_FORMAT : Fail to get value.");
        error = true;
    }
    else {
        try {
            EParser<CamPixFmt> camPixFmt;
            m_Param.camInput.ACQ_FORMAT = camPixFmt.parseEnum("ACQ_FORMAT", pixfmt);
        }
        catch (exception& ex) {
            m_Param.camInput.errormsg.push_back(" ACQ_FORMAT : " + string(ex.what()));
            error = true;
        }
    }

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("ACQ_RES_CUSTOM_SIZE", m_Param.camInput.ACQ_RES_CUSTOM_SIZE)) {
        m_Param.camInput.errormsg.push_back(" ACQ_RES_CUSTOM_SIZE : Fail to get value.");
        error = true;
    }
    else {

        if (m_Param.camInput.ACQ_RES_CUSTOM_SIZE) {
            string acq_offset;
            if (!m_Cfg.Get("ACQ_OFFSET", acq_offset)) {
                m_Param.camInput.errormsg.push_back(" ACQ_OFFSET : Fail to get value.");
                error = true;
            }
            else {

                if (acq_offset.find(",") != string::npos) {
                    //cout << "++++++++++++++++++++++++++++++++++++++ FIND ACQ OFFSET " << acq_offset << endl;
                    string offsetx = acq_offset.substr(0, acq_offset.find(","));
                    string offsety = acq_offset.substr(acq_offset.find(",") + 1, string::npos);
                    int mStartX = atoi(offsetx.c_str());
                    int mStartY = atoi(offsety.c_str());

                    if (mStartX <= 0) {
                        m_Param.camInput.errormsg.push_back(" ACQ_OFFSET : X offset value is not correct.");
                        error = true;
                    }
                    else {
                        m_Param.camInput.ACQ_STARTX = mStartX;

                    }

                    if (mStartY <= 0) {
                        m_Param.camInput.errormsg.push_back(" ACQ_OFFSET : Y offset value is not correct.");
                        error = true;
                    }
                    else {
                        m_Param.camInput.ACQ_STARTY = mStartY;
                    }

                }
                else {
                    m_Param.camInput.errormsg.push_back(" ACQ_OFFSET : Format is not correct. It must be : X,Y.");
                    error = true;
                }
            }

            string acq_res_custome_size;
            if (!m_Cfg.Get("ACQ_RES_SIZE", acq_res_custome_size)) {
                m_Param.camInput.errormsg.push_back(" ACQ_RES_SIZE : Fail to get value.");
                error = true;
            }
            else {

                if (acq_res_custome_size.find("x") != string::npos) {
                    //cout << "++++++++++++++++++++++++++++++++++++++ FIND ACQ RES SIZE " << acq_res_custome_size << endl;
                    string width = acq_res_custome_size.substr(0, acq_res_custome_size.find("x"));
                    string height = acq_res_custome_size.substr(acq_res_custome_size.find("x") + 1, string::npos);
                    int mSizeWidth = atoi(width.c_str());
                    int mSizeHeight = atoi(height.c_str());

                    if (mSizeHeight <= 0) {
                        m_Param.camInput.errormsg.push_back(" ACQ_RES_SIZE : Height value is not correct.");
                        error = true;
                    }
                    else {
                        //cout << "++++++++++++++++++++++++++++++++++++++ ACQ_HEIGHT " << mSizeHeight << endl;
                        m_Param.camInput.ACQ_HEIGHT = mSizeHeight;
                    }

                    if (mSizeWidth <= 0) {
                        m_Param.camInput.errormsg.push_back(" ACQ_RES_SIZE : Width value is not correct.");
                        error = true;
                    }
                    else {
                        m_Param.camInput.ACQ_WIDTH = mSizeWidth;
                    }

                }
                else {
                    m_Param.camInput.errormsg.push_back(" ACQ_RES_SIZE : Format is not correct. It must be : WxH.");
                    error = true;
                }
            }
        }
        else {

            m_Param.camInput.ACQ_STARTX = 0;
            m_Param.camInput.ACQ_STARTY = 0;
            m_Param.camInput.ACQ_HEIGHT = 1080;
            m_Param.camInput.ACQ_WIDTH = 1440;
        }
    }

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("SHIFT_BITS", m_Param.camInput.SHIFT_BITS)) {
        m_Param.camInput.errormsg.push_back(" SHIFT_BITS : Fail to get value.");
        error = true;
    }

//     double ming = -1, maxg = -1;
//     double mine = -1, maxe = -1;
//     double minf = -1, maxf = -1;
// 
// //     device->getCameraFPSBounds(minf, maxf);
// //     device->setCameraDayGain();
// //     device->getCameraGainBounds(ming, maxg);
// //     device->getCameraExposureBounds(mine, maxe);

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("ACQ_NIGHT_EXPOSURE", m_Param.camInput.ACQ_NIGHT_EXPOSURE)) {
        m_Param.camInput.errormsg.push_back(" ACQ_NIGHT_EXPOSURE : Fail to get value.");
        error = true;
    }
//     else {
// 
//         if (mine != -1 && maxe != -1) {
//             if (m_Param.camInput.ACQ_NIGHT_EXPOSURE < mine || m_Param.camInput.ACQ_NIGHT_EXPOSURE > maxe) {
//                 m_Param.camInput.errormsg.push_back(" ACQ_NIGHT_EXPOSURE : Value <" +
//                     Conversion::intToString(m_Param.camInput.ACQ_NIGHT_EXPOSURE) +
//                     "> is not correct. \nAvailable range is from " +
//                     Conversion::intToString(mine) + " to " +
//                     Conversion::intToString(maxe));
//                 error = true;
//             }
//         }
// 
//         LOG_INFO << "EXP NIGHT " << m_Param.camInput.ACQ_NIGHT_EXPOSURE << endl;
//     }

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("ACQ_NIGHT_GAIN", m_Param.camInput.ACQ_NIGHT_GAIN)) {
        m_Param.camInput.errormsg.push_back(" ACQ_NIGHT_GAIN : Fail to get value.");
        error = true;
    }
//     else {
// 
//         if (ming != -1 && maxg != -1) {
//             if (m_Param.camInput.ACQ_NIGHT_GAIN < ming || m_Param.camInput.ACQ_NIGHT_GAIN > maxg) {
//                 m_Param.camInput.errormsg.push_back(" ACQ_NIGHT_GAIN : Value <" +
//                     Conversion::intToString(m_Param.camInput.ACQ_NIGHT_GAIN) +
//                     "> is not correct. \nAvailable range is from " +
//                     Conversion::intToString(ming) + " to " +
//                     Conversion::intToString(maxg));
//                 error = true;
//             }
//         }
//     }

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("ACQ_DAY_EXPOSURE", m_Param.camInput.ACQ_DAY_EXPOSURE)) {
        m_Param.camInput.errormsg.push_back(" ACQ_DAY_EXPOSURE : Fail to get value.");
        error = true;
    }
//     else {
// 
//         if (mine != -1 && maxe != -1) {
//             if (m_Param.camInput.ACQ_DAY_EXPOSURE < mine || m_Param.camInput.ACQ_DAY_EXPOSURE > maxe) {
//                 m_Param.camInput.errormsg.push_back(" ACQ_DAY_EXPOSURE : Value <" +
//                     Conversion::intToString(m_Param.camInput.ACQ_DAY_EXPOSURE) +
//                     "> is not correct. \nAvailable range is from " +
//                     Conversion::intToString(mine) + " to " +
//                     Conversion::intToString(maxe));
//                 error = true;
//             }
//         }
//     }

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("ACQ_DAY_GAIN", m_Param.camInput.ACQ_DAY_GAIN)) {
        m_Param.camInput.errormsg.push_back(" ACQ_DAY_GAIN : Fail to get value.");
        error = true;
    }
//     else {
// 
//         if (ming != -1 && maxg != -1) {
//             if (m_Param.camInput.ACQ_DAY_GAIN < ming || m_Param.camInput.ACQ_DAY_GAIN > maxg) {
//                 m_Param.camInput.errormsg.push_back(" ACQ_DAY_GAIN : Value <" +
//                     Conversion::intToString(m_Param.camInput.ACQ_DAY_GAIN) +
//                     "> is not correct. \nAvailable range is from " +
//                     Conversion::intToString(ming) + " to " +
//                     Conversion::intToString(maxg));
//                 error = true;
//             }
//         }
//     }

    //-------------------------------------------------------------------
    if (!m_Cfg.Get("EXPOSURE_CONTROL_ENABLED", m_Param.camInput.EXPOSURE_CONTROL_ENABLED)) {
        m_Param.camInput.errormsg.push_back(" EXPOSURE_CONTROL_ENABLED : Fail to get value.");
        error = true;
    }
    else {
        if (m_Param.camInput.EXPOSURE_CONTROL_ENABLED) {


            if (!m_Cfg.Get("EXPOSURE_CONTROL_FREQUENCY", m_Param.camInput.EXPOSURE_CONTROL_FREQUENCY)) {
                m_Param.camInput.errormsg.push_back(" EXPOSURE_CONTROL_FREQUENCY : Fail to get value.");
                error = true;
            }

            //-------------------------------------------------------------------

            if (!m_Cfg.Get("EXPOSURE_CONTROL_SAVE_IMAGE", m_Param.camInput.EXPOSURE_CONTROL_SAVE_IMAGE)) {
                m_Param.camInput.errormsg.push_back(" EXPOSURE_CONTROL_SAVE_IMAGE : Fail to get value.");
                error = true;
            }

            //-------------------------------------------------------------------

            if (!m_Cfg.Get("EXPOSURE_CONTROL_SAVE_INFOS", m_Param.camInput.EXPOSURE_CONTROL_SAVE_INFOS)) {
                m_Param.camInput.errormsg.push_back(" EXPOSURE_CONTROL_SAVE_INFOS : Fail to get value.");
                error = true;
            }
        }
    }

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("EPHEMERIS_ENABLED", m_Param.camInput.ephem.EPHEMERIS_ENABLED)) {
        m_Param.camInput.errormsg.push_back(" EPHEMERIS_ENABLED : Fail to get value.");
        error = true;
    }

    //-------------------------------------------------------------------
    if (m_Param.camInput.ephem.EPHEMERIS_ENABLED) {
        if (!m_Cfg.Get("SUN_HORIZON_1", m_Param.camInput.ephem.SUN_HORIZON_1)) {
            m_Param.camInput.errormsg.push_back(" SUN_HORIZON_1 : Fail to get value.");
            error = true;
        }

        //-------------------------------------------------------------------

        if (!m_Cfg.Get("SUN_HORIZON_2", m_Param.camInput.ephem.SUN_HORIZON_2)) {
            m_Param.camInput.errormsg.push_back(" SUN_HORIZON_2 : Fail to get value.");
            error = true;
        }
    }

    //-------------------------------------------------------------------

    if (!m_Param.camInput.ephem.EPHEMERIS_ENABLED) {

        string sunrise_time;
        if (!m_Cfg.Get("SUNRISE_TIME", sunrise_time)) {
            m_Param.camInput.errormsg.push_back(" SUNRISE_TIME : Fail to get value.");
            error = true;
        }
        else {

            if (sunrise_time.find(":") != string::npos) {

                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                boost::char_separator<char> sep(":");
                tokenizer tokens(sunrise_time, sep);

                m_Param.camInput.ephem.SUNRISE_TIME.clear();
                for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
                    m_Param.camInput.ephem.SUNRISE_TIME.push_back(atoi((*tok_iter).c_str()));
                }

                if (m_Param.camInput.ephem.SUNRISE_TIME.size() == 2) {
                    if (m_Param.camInput.ephem.SUNRISE_TIME.at(0) < 0 || m_Param.camInput.ephem.SUNRISE_TIME.at(0) >= 24) {
                        m_Param.camInput.errormsg.push_back(" SUNRISE_TIME : Hours value must be between 0 - 23");
                        error = true;
                    }

                    if (m_Param.camInput.ephem.SUNRISE_TIME.at(1) < 0 || m_Param.camInput.ephem.SUNRISE_TIME.at(0) >= 60) {
                        m_Param.camInput.errormsg.push_back(" SUNRISE_TIME : Minutes value must be between 0 - 59");
                        error = true;
                    }
                }

            }
            else {
                m_Param.camInput.errormsg.push_back(" SUNRISE_TIME : Format is not correct. It must be : HH:MM");
                error = true;
            }
        }

        //-------------------------------------------------------------------

        string sunset_time;
        if (!m_Cfg.Get("SUNSET_TIME", sunset_time)) {
            m_Param.camInput.errormsg.push_back(" SUNSET_TIME : Fail to get value.");
            error = true;
        }
        else {

            if (sunset_time.find(":") != string::npos) {

                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                boost::char_separator<char> sep(":");
                tokenizer tokens(sunset_time, sep);

                m_Param.camInput.ephem.SUNSET_TIME.clear();
                for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
                    m_Param.camInput.ephem.SUNSET_TIME.push_back(atoi((*tok_iter).c_str()));

                if (m_Param.camInput.ephem.SUNSET_TIME.size() == 2) {
                    if (m_Param.camInput.ephem.SUNSET_TIME.at(0) < 0 || m_Param.camInput.ephem.SUNSET_TIME.at(0) >= 24) {
                        m_Param.camInput.errormsg.push_back(" SUNSET_TIME : Hours value must be between 0 - 23");
                        error = true;
                    }

                    if (m_Param.camInput.ephem.SUNSET_TIME.at(1) < 0 || m_Param.camInput.ephem.SUNSET_TIME.at(0) >= 60) {
                        m_Param.camInput.errormsg.push_back(" SUNSET_TIME : Minutes value must be between 0 - 59");
                        error = true;
                    }
                }

            }
            else {
                m_Param.camInput.errormsg.push_back(" SUNSET_TIME : Format is not correct. It must be : HH:MM");
                error = true;
            }
        }

        //-------------------------------------------------------------------

        if (!m_Cfg.Get("SUNSET_DURATION", m_Param.camInput.ephem.SUNSET_DURATION)) {
            m_Param.camInput.errormsg.push_back(" SUNSET_DURATION : Fail to get value.");
            error = true;
        }

        //-------------------------------------------------------------------

        if (!m_Cfg.Get("SUNRISE_DURATION", m_Param.camInput.ephem.SUNRISE_DURATION)) {
            m_Param.camInput.errormsg.push_back(" SUNRISE_DURATION : Fail to get value.");
            error = true;
        }
    }

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("ACQ_REGULAR_ENABLED", m_Param.camInput.regcap.ACQ_REGULAR_ENABLED)) {
        m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_ENABLED : Fail to get value.");
        error = true;
    }
    else {
        if (m_Param.camInput.regcap.ACQ_REGULAR_ENABLED) {
            //-------------------------------------------------------------------

            string reg_mode;
            if (!m_Cfg.Get("ACQ_REGULAR_MODE", reg_mode)) {
                error = true;
                m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_MODE : Fail to load value.");
            }
            else {
                try {
                    EParser<TimeMode> regMode;
                    m_Param.camInput.regcap.ACQ_REGULAR_MODE = regMode.parseEnum("ACQ_REGULAR_MODE", reg_mode);
                }
                catch (exception& ex) {
                    error = true;
                    m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_MODE : " + string(ex.what()));
                }
            }

            //-------------------------------------------------------------------

            {

                string img_prefix;
                if (!m_Cfg.Get("ACQ_REGULAR_PRFX", img_prefix)) {
                    error = true;
                    m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_PRFX : Fail to load value.");
                }
                else {
                    m_Param.camInput.regcap.ACQ_REGULAR_PRFX = img_prefix;
                }
            }

            //-------------------------------------------------------------------
            {

                string img_output;
                if (!m_Cfg.Get("ACQ_REGULAR_OUTPUT", img_output)) {
                    error = true;
                    m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_OUTPUT : Fail to load value.");
                }
                else {
                    try {
                        EParser<ImgFormat> imgOutput;
                        m_Param.camInput.regcap.ACQ_REGULAR_OUTPUT = imgOutput.parseEnum("ACQ_REGULAR_OUTPUT", img_output);
                    }
                    catch (exception& ex) {
                        error = true;
                        m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_OUTPUT : " + string(ex.what()));
                    }
                }
            }

            //-------------------------------------------------------------------

            string regAcqParam;
            if (!m_Cfg.Get("ACQ_REGULAR_CFG", regAcqParam)) {
                error = true;
                m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_CFG : Fail to load value.");
            }
            else {
                transform(regAcqParam.begin(), regAcqParam.end(), regAcqParam.begin(), ::toupper);

                typedef boost::tokenizer<boost::char_separator<char> > tokenizer1;
                boost::char_separator<char> sep1("HMSEGFN");
                tokenizer1 tokens1(regAcqParam, sep1);

                vector<string> res1;
                for (tokenizer1::iterator tokIter = tokens1.begin(); tokIter != tokens1.end(); ++tokIter)
                    res1.push_back(*tokIter);

                if (res1.size() == 7) {

                    // Get regular acquisition time interval.
                    if (atoi(res1.at(0).c_str()) < 0 || atoi(res1.at(0).c_str()) >= 24) {
                        error = true;
                        m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_CFG : Hours can't have the value <" + res1.at(0) + ">.\nAvailable range is from 0 to 23.");
                    }

                    if (atoi(res1.at(1).c_str()) < 0 || atoi(res1.at(1).c_str()) >= 60) {
                        error = true;
                        m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_CFG : Minutes can't have the value <" + res1.at(1) + ">.\nAvailable range is from 0 to 23.");
                    }

                    if (atoi(res1.at(2).c_str()) < 0 || atoi(res1.at(2).c_str()) >= 60) {
                        error = true;
                        m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_CFG : Seconds can't have the value <" + res1.at(2) + ">.\nAvailable range is from 0 to 23.");
                    }

                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.interval = atoi(res1.at(0).c_str()) * 3600 + atoi(res1.at(1).c_str()) * 60 + atoi(res1.at(2).c_str());

                    // Get regular acquisition exposure time.
                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.exp = atoi(res1.at(3).c_str());

//                     if (mine != -1 && maxe != -1) {
//                         if (m_Param.camInput.regcap.ACQ_REGULAR_CFG.exp < mine || m_Param.camInput.regcap.ACQ_REGULAR_CFG.exp > maxe) {
//                             m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_CFG : Exposure value <" +
//                                 Conversion::intToString(m_Param.camInput.regcap.ACQ_REGULAR_CFG.exp) +
//                                 "> is not correct. \nAvailable range is from " +
//                                 Conversion::intToString(mine) + " to " +
//                                 Conversion::intToString(maxe));
//                             error = true;
//                         }
//                     }

                    // Get regular acquisition gain.
                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.gain = atoi(res1.at(4).c_str());
// 
//                     if (ming != -1 && maxg != -1) {
//                         if (m_Param.camInput.regcap.ACQ_REGULAR_CFG.gain < ming || m_Param.camInput.regcap.ACQ_REGULAR_CFG.gain > maxg) {
//                             m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_CFG : Gain value <" +
//                                 Conversion::intToString(m_Param.camInput.regcap.ACQ_REGULAR_CFG.gain) +
//                                 "> is not correct. \nAvailable range is from " +
//                                 Conversion::intToString(ming) + " to " +
//                                 Conversion::intToString(maxg));
//                             error = true;
//                         }
//                     }

                    // Get regular acquisition repetition.
                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.rep = atoi(res1.at(6).c_str());

                    // Get regular acquisition format.
                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.fmt = static_cast<CamPixFmt>(atoi(res1.at(5).c_str()));
                    EParser<CamPixFmt> fmt;
                    if (fmt.getStringEnum(m_Param.camInput.regcap.ACQ_REGULAR_CFG.fmt) == "") {
                        error = true;
                        m_Param.camInput.errormsg.push_back(" ACQ_REGULAR_CFG : Fail to extract pixel format on " + regAcqParam + ". Check if index <" + res1.at(5) + "> exits.");
                    }
                }
            }
        }
    }

    //-------------------------------------------------------------------

    if (!m_Cfg.Get("ACQ_SCHEDULE_ENABLED", m_Param.camInput.schcap.ACQ_SCHEDULE_ENABLED)) {
        m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE_ENABLED : Fail to get value.");
        error = true;
    }
    else {

        if (m_Param.camInput.schcap.ACQ_SCHEDULE_ENABLED) {

            if (!m_Param.camInput.regcap.ACQ_REGULAR_ENABLED) {
                //-------------------------------------------------------------------

                {
                    string img_output;
                    if (!m_Cfg.Get("ACQ_SCHEDULE_OUTPUT", img_output)) {
                        error = true;
                        m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE_OUTPUT : Fail to load value.");
                    }
                    else {
                        try {
                            EParser<ImgFormat> imgOutput;
                            m_Param.camInput.schcap.ACQ_SCHEDULE_OUTPUT = imgOutput.parseEnum("ACQ_SCHEDULE_OUTPUT", img_output);
                        }
                        catch (exception& ex) {
                            error = true;
                            m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE_OUTPUT : " + string(ex.what()));
                        }
                    }
                }

                //-------------------------------------------------------------------

                {
                    string sACQ_SCHEDULE;
                    if (!m_Cfg.Get("ACQ_SCHEDULE", sACQ_SCHEDULE)) {
                        error = true;
                        m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : Fail to load value.");
                    }
                    else {

                        vector<string> sch1;

                        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                        boost::char_separator<char> sep(",");
                        tokenizer tokens(sACQ_SCHEDULE, sep);

                        for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
                            string s = *tok_iter;
                            transform(s.begin(), s.end(), s.begin(), ::toupper);
                            sch1.push_back(s);
                        }

                        for (int i = 0; i < sch1.size(); i++) {

                            typedef boost::tokenizer<boost::char_separator<char> > tokenizer_;
                            boost::char_separator<char> sep_("HMSEGFN");
                            tokenizer tokens_(sch1.at(i), sep_);

                            vector<string> sp;

                            for (tokenizer::iterator tok_iter_ = tokens_.begin(); tok_iter_ != tokens_.end(); ++tok_iter_)
                                sp.push_back(*tok_iter_);

                            if (sp.size() == 7) {

                                scheduleParam spa;
                                bool status = true;
                                spa.hours = atoi(sp.at(0).c_str());
                                if (spa.hours < 0 || spa.hours >= 24) {
                                    error = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : In " + sch1.at(i) + ". Hours can't have the value <" + Conversion::intToString(spa.hours) + ">.\nAvailable range is from 0 to 23.");
                                }
                                spa.min = atoi(sp.at(1).c_str());
                                if (spa.min < 0 || spa.min >= 60) {
                                    error = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : In " + sch1.at(i) + ". Minutes can't have the value <" + Conversion::intToString(spa.min) + ">.\nAvailable range is from 0 to 59.");
                                }
                                spa.sec = atoi(sp.at(2).c_str());
                                if (spa.sec < 0 || spa.sec >= 60) {
                                    error = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : In " + sch1.at(i) + ". Seconds can't have the value <" + Conversion::intToString(spa.sec) + ">.\nAvailable range is from 0 to 59.");
                                }
                                spa.exp = atoi(sp.at(3).c_str());

//                                 if (mine != -1 && maxe != -1) {
//                                     if (spa.exp < mine || spa.exp > maxe) {
//                                         m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : In " + sch1.at(i) + ". Exposure value <" +
//                                             Conversion::intToString(spa.exp) +
//                                             "> is not correct. \nAvailable range is from " +
//                                             Conversion::intToString(mine) + " to " +
//                                             Conversion::intToString(maxe));
//                                         error = true;
//                                         status = false;
//                                     }
//                                 }

                                spa.gain = atoi(sp.at(4).c_str());

//                                 if (ming != -1 && maxg != -1) {
//                                     if (spa.gain < ming || spa.gain > maxg) {
//                                         m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : In " + sch1.at(i) + ". Gain value <" +
//                                             Conversion::intToString(spa.gain) +
//                                             "> is not correct. \nAvailable range is from " +
//                                             Conversion::intToString(ming) + " to " +
//                                             Conversion::intToString(maxg));
//                                         error = true;
//                                         status = false;
//                                     }
//                                 }

                                spa.rep = atoi(sp.at(6).c_str());
                                if (spa.rep < 0 || spa.rep >= 60) {
                                    error = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : One repetition must be defined at least.");
                                }
                                spa.fmt = static_cast<CamPixFmt>(atoi(sp.at(5).c_str()));
                                EParser<CamPixFmt> fmt;
                                if (fmt.getStringEnum(spa.fmt) == "") {
                                    error = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : Fail to extract pixel format for :  " + sch1.at(i) + ". Index <" + sp.at(5) + "> not exist.");
                                }

                                if (status)
                                    m_Param.camInput.schcap.ACQ_SCHEDULE.push_back(spa);

                            }
                        }

                        // Order scheduled acquisition

                        if (m_Param.camInput.schcap.ACQ_SCHEDULE.size() != 0) {

                            // Sort time in list.
                            vector<scheduleParam> tempSchedule;

                            do {

                                int minH; int minM; int minS; bool init = false;

                                vector<scheduleParam>::iterator it;
                                vector<scheduleParam>::iterator it_select;

                                for (it = m_Param.camInput.schcap.ACQ_SCHEDULE.begin(); it != m_Param.camInput.schcap.ACQ_SCHEDULE.end(); ++it) {

                                    if (!init) {

                                        minH = (*it).hours;
                                        minM = (*it).min;
                                        minS = (*it).sec;
                                        it_select = it;
                                        init = true;

                                    }
                                    else {

                                        if ((*it).hours < minH) {

                                            minH = (*it).hours;
                                            minM = (*it).min;
                                            minS = (*it).sec;
                                            it_select = it;

                                        }
                                        else if ((*it).hours == minH) {

                                            if ((*it).min < minM) {

                                                minH = (*it).hours;
                                                minM = (*it).min;
                                                minS = (*it).sec;
                                                it_select = it;

                                            }
                                            else if ((*it).min == minM) {

                                                if ((*it).sec < minS) {

                                                    minH = (*it).hours;
                                                    minM = (*it).min;
                                                    minS = (*it).sec;
                                                    it_select = it;

                                                }

                                            }

                                        }

                                    }

                                }

                                if (init) {

                                    tempSchedule.push_back((*it_select));
                                    //cout << "> " << (*it_select).hours << "H " << (*it_select).min << "M " << (*it_select).sec << "S " << endl;
                                    m_Param.camInput.schcap.ACQ_SCHEDULE.erase(it_select);

                                }

                            } while (m_Param.camInput.schcap.ACQ_SCHEDULE.size() != 0);

                            m_Param.camInput.schcap.ACQ_SCHEDULE = tempSchedule;

                        }
                    }
                }

            }
            else {
                error = true;
                m_Param.camInput.errormsg.push_back(" ACQ_SCHEDULE : Disable ACQ_REGULAR_ENABLED to use ACQ_SCHEDULE.");
            }
        }
    }

    if (!error) m_Param.camInput.status = true;
}

bool CfgParam::checkInputParam(InputDeviceType m_InputType) 
{
    switch (m_InputType) {

    case VIDEO:
        return checkVidParam();
        break;

    case SINGLE_FITS_FRAME:
        return checkFramesParam();
        break;

        // camera
    case CAMERA:
        return checkCamParam();
        break;

    }

    return false;
}

void CfgParam::loadInputParam(InputDeviceType m_InputType)
{
    switch (m_InputType)
    {

    case VIDEO:
        loadVidParam();
        break;

    case SINGLE_FITS_FRAME:
        loadFramesParam();
        break;

    case CAMERA:
        loadCamParam();
        break;
    };
}
