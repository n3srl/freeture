/*
                            CfgParam.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2016 Yoan Audureau, Chiara Marmo
*                       2017-2018 Chiara Marmo
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
*   Last modified:      20/03/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    CfgParam.cpp
* \author  Yoan Audureau -- Chiara Marmo -- GEOPS-UPSUD
* \version 1.2
* \date    20/03/2018
* \brief   Get parameters from configuration file.
*/


#include <fstream>
#include <string>
#include <iostream>
#include <map>
#include <stdlib.h>

#include <filesystem>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>

#include "ECamPixFmt.h"
#include "ETimeMode.h"
#include "EImgFormat.h"
#include "EDetMeth.h"
#include "ELogSeverityLevel.h"
#include "EStackMeth.h"
#include "ESmtpSecurity.h"
#include "CfgLoader.h"
#include "Device.h"
#include "SParam.h"
#include "ECamSdkType.h"
#include "CfgParam.h"
#include "Logger.h"



using namespace boost::filesystem;
using namespace std;
using namespace cv;

boost::log::sources::severity_logger< LogSeverityLevel >  freeture::CfgParam::m_Logger;

freeture::CfgParam::Init freeture::CfgParam::initializer;

freeture::CfgParam::CfgParam(Device* device, string cfgFilePath)
{
    assert(device!=nullptr);
    manager = CameraDeviceManager::Get();
    m_CfgFilePath = cfgFilePath;
    // Initialize parameters.
    mDevice = device;

    showErrors = false;

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
    m_Param.log.LOG_SEVERITY = notification;
    m_Param.log.LOG_SIZE_LIMIT = 50;

    vector<string> finput, vinput;
    m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH = finput;
    m_Param.vidInput.INPUT_VIDEO_PATH = vinput;
    m_Param.framesInput.INPUT_TIME_INTERVAL = 0;
    m_Param.vidInput.INPUT_TIME_INTERVAL = 0;

    m_Param.camInput.ACQ_DAY_EXPOSURE = 0;
    m_Param.camInput.ACQ_DAY_GAIN = 0;
    m_Param.camInput.ACQ_FORMAT = MONO8;
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

    boost::filesystem::path pcfg(cfgFilePath);
    if(boost::filesystem::exists(pcfg)) {
        if(m_Cfg.Load(cfgFilePath)) {
            loadCameraSerial();
            loadCameraInit();
            loadDeviceID();
            loadDataParam();
            loadLogParam();

            if(m_Param.DEVICE_ID != -1) {

                CamSdkType sdk = manager.getDevice()->getDeviceSdk(m_Param.DEVICE_ID);
                m_InputType = manager.getDevice()->getDeviceType(sdk);

                switch(m_InputType)
                {

                    case VIDEO :
                            loadVidParam();
                        break;

                    case SINGLE_FITS_FRAME :
                            loadFramesParam();
                        break;

                    // camera
                    case CAMERA :
                            loadCamParam();
                        break;

                }


            }

            loadDetParam();
            loadStackParam();
            loadStationParam();
            loadFitskeysParam();
            loadMailParam();
        }else{
            m_EMsg.push_back("Fail to load configuration file.");
            cout << "Fail to load configuration file." << endl;
        }
    }else{
        m_EMsg.push_back("Configuration file path not exists : " + cfgFilePath);
        //cout << "Configuration file path not exists : " << cfgFilePath << endl;
        freeture::LogError("Configuration file path not exists : " , cfgFilePath);
    }
}
void freeture::CfgParam::setInitRequired(bool init)
{
    std::cout << "Set init req" << std::endl;
    FILE* ofile = fopen(FREETURE_WRITE_CONFIG_PATH, "w");
    std::ifstream file(m_CfgFilePath);
    if(!file.is_open())
    {
        std::cout << "Impossibile resettare il file di configurazione" << std::endl;
        return;
    }

    std::string line;
    while (!file.eof())
    {
        std::getline(file, line);
        size_t found = line.find("CAMERA_INIT = true");
        if(found != std::string::npos)
        {
            line = "CAMERA_INIT = false";
        }
        //std::cout << "Write line " << line << std::endl;
        line += '\n';
        fprintf(ofile, "%s", line.c_str());
    }
    fclose(ofile);
    file.close();
    const char* newFileName = (m_CfgFilePath  + ".old").c_str();
    std::rename(m_CfgFilePath.c_str(), newFileName);
    std::rename(FREETURE_WRITE_CONFIG_PATH, m_CfgFilePath.c_str());
    std::remove(newFileName);
    return;
}
void freeture::CfgParam::loadCameraInit()
{
    
    std::string defaultCameraConfigFile = "/freeture/cinit.cfg";
    bool cBool = false;
    if(!m_Cfg.Get("CAMERA_INIT", cBool))
    {
        m_Param.CAMERA_INIT = false;
    } else 
    {
        m_Param.CAMERA_INIT = cBool;
    } 
    

    std::string cString = "";
    if(!m_Cfg.Get("CAMERA_INIT_CONFIG", cString))
    {
        m_Param.CAMERA_INIT_CONFIG = defaultCameraConfigFile;
    } 
    else {
        m_Param.CAMERA_INIT_CONFIG = cString;
    }
    
    if(m_Param.CAMERA_INIT_CONFIG == "") m_Param.CAMERA_INIT_CONFIG = defaultCameraConfigFile;

    std::string message;
    if(m_Param.CAMERA_INIT)
    {
        bool initDone;

        if(FILE* file = fopen(FREETURE_CHECK_INIT_DONE, "r")) {
            fclose(file);
            initDone = true;
        } else 
        {
            initDone = false;
        }

        if(initDone)
        {
            setInitRequired(true);
            std::remove(FREETURE_CHECK_INIT_DONE);
        }
        message = "YES";
        std::cout << "Camera init file " << m_Param.CAMERA_INIT_CONFIG << std::endl;
    } else 
    {
        message = "NO";
    }
    std::cout << "Need to init camera: " << message << std::endl;


}

void freeture::CfgParam::loadCameraSerial()
{
    manager.listDevice(false);
    std::string cString;

    bool failStringSerial = false;
    std::string failmsg = "- CAMERA_SERIAL : ";

    if(!m_Cfg.Get("CAMERA_SERIAL", cString))
    {
        failStringSerial = true;
        failmsg += "Fail to get camera seria, DeviceId will be used for camera selection";
    }

    m_Param.CAMERA_SERIAL = cString;
}

void freeture::CfgParam::loadDeviceID() {
    manager.listDevice(true);
    int cId;
    string cString;
    bool failIntId, failStringId = false;
    string failmsg = "- CAMERA_ID : ";

    if(!m_Cfg.Get("CAMERA_ID", cId)) {
        failIntId = true;
        failmsg += "Fail to get value. Probably not defined.\n";
    }

    else if(!m_Cfg.Get("CAMERA_ID", cString)) {
        failStringId = true;
        failmsg += "Fail to get value. Probably not defined.\n";
    }

    else {
        try {
            EParser<CamSdkType> cam_string;
            CamSdkType cType = manager.getDevice()->getDeviceSdk(cId);
        } catch (std::exception &ex){
        }


    }
    if (manager.deviceNumber < 0 || cId > (manager.deviceNumber - 1)) {
        std::cout << "- CAMERA_ID's value not exist." << std::endl;
        m_Param.DEVICE_ID = -1;
        return;
    }

    m_Param.DEVICE_ID = cId;
    
}

void freeture::CfgParam::loadDataParam() {

    bool e = false;

    if(!m_Cfg.Get("DATA_PATH", m_Param.data.DATA_PATH)) {
        m_Param.data.errormsg.push_back("- DATA_PATH : Fail to get value.");
        e = true;
    }else{

        namespace fs = boost::filesystem;
        path p(m_Param.data.DATA_PATH);

        if(!fs::exists(p)){
            try {
                fs::create_directory(p);
            } catch (std::exception &ex) {
                m_Param.data.errormsg.push_back("- DATA_PATH : Can't create Data Path directory.");
                e = true;
            }
        }
    }

    if(!m_Cfg.Get("FITS_COMPRESSION", m_Param.data.FITS_COMPRESSION)) {
        m_Param.data.errormsg.push_back("- FITS_COMPRESSION : Fail to get value.");
        e = true;
    }else{

        m_Param.data.FITS_COMPRESSION_METHOD = "";

        if(m_Param.data.FITS_COMPRESSION) {
            if(!m_Cfg.Get("FITS_COMPRESSION_METHOD", m_Param.data.FITS_COMPRESSION_METHOD)) {
                m_Param.data.errormsg.push_back("- FITS_COMPRESSION_METHOD : Fail to get value.");
                e = true;
            }
        }
    }

    if(!e) m_Param.data.status = true;
}

void freeture::CfgParam::loadLogParam() {

    bool e = false;

    if(!m_Cfg.Get("LOG_PATH", m_Param.log.LOG_PATH)) {
        m_Param.log.errormsg.push_back("- LOG_PATH : Fail to get value.");
        e = true;
    }else{

        namespace fs = boost::filesystem;
        path p(m_Param.log.LOG_PATH);

        if(!fs::exists(p)){
            try {
                fs::create_directory(p);
            } catch (std::exception &ex) {
                m_Param.log.errormsg.push_back("- LOG_PATH : Can't create Log Path directory.");
                e = true;
            }
        }
    }

    if(!m_Cfg.Get("LOG_ARCHIVE_DAY", m_Param.log.LOG_ARCHIVE_DAY)) {
        m_Param.log.errormsg.push_back("- LOG_ARCHIVE_DAY : Fail to get value.");
        e = true;
    }

    if(!m_Cfg.Get("LOG_SIZE_LIMIT", m_Param.log.LOG_SIZE_LIMIT)) {
        m_Param.log.errormsg.push_back("- LOG_SIZE_LIMIT : Fail to get value.");
        e = true;
    }

    string log_severity;
    EParser<LogSeverityLevel> log_sev;
    if(!m_Cfg.Get("LOG_SEVERITY", log_severity)) {
        m_Param.log.errormsg.push_back("- LOG_SEVERITY : Fail to get value.");
        e = true;
    }

    try {
        m_Param.log.LOG_SEVERITY = log_sev.parseEnum("LOG_SEVERITY", log_severity);
    }catch (std::exception &ex) {
        m_Param.log.errormsg.push_back("- LOG_SEVERITY : " + string(ex.what()));
        e = true;
    }

    if(!e) m_Param.log.status = true;
}

void freeture::CfgParam::loadFramesParam() {

    bool e = false;

    if(!m_Cfg.Get("INPUT_TIME_INTERVAL", m_Param.framesInput.INPUT_TIME_INTERVAL)) {
        m_Param.framesInput.errormsg.push_back("- INPUT_TIME_INTERVAL : Fail to get value.");
        //cout << "- INPUT_FRAMES_DIRECTORY_PATH : Fail to get value." << endl;
        e = true;
    }

    string inputPaths;
    if(!m_Cfg.Get("INPUT_FRAMES_DIRECTORY_PATH", inputPaths)) {
        m_Param.framesInput.errormsg.push_back("- INPUT_FRAMES_DIRECTORY_PATH : Fail to get value.");
        //cout << "- INPUT_FRAMES_DIRECTORY_PATH : Fail to get value." << endl;
        e = true;
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(",");
    tokenizer tokens(inputPaths, sep);

    for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter){
        boost::filesystem::path p_input_frames_dir(*tok_iter);
        if(!boost::filesystem::exists(p_input_frames_dir)) {
            m_Param.framesInput.errormsg.push_back("- INPUT_FRAMES_DIRECTORY_PATH : " + *tok_iter + " not exists.");
            e = true;
        }else{
            m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.push_back(*tok_iter);
        }
    }

    if(!e) m_Param.framesInput.status = true;
}

void freeture::CfgParam::loadVidParam() {

    bool e = false;

    if(!m_Cfg.Get("INPUT_TIME_INTERVAL", m_Param.vidInput.INPUT_TIME_INTERVAL)) {
        m_Param.vidInput.errormsg.push_back("- INPUT_TIME_INTERVAL : Fail to get value.");
        //cout << "- INPUT_FRAMES_DIRECTORY_PATH : Fail to get value." << endl;
        e = true;
    }

    string input_video_path;
    if(!m_Cfg.Get("INPUT_VIDEO_PATH", input_video_path)) {
        m_Param.vidInput.errormsg.push_back("- INPUT_VIDEO_PATH : Fail to get value.");
        e = true;
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(",");
    tokenizer tokens(input_video_path, sep);

    for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter){
        boost::filesystem::path p_input_video_path(*tok_iter);
        if(!is_regular_file(p_input_video_path)) {
            m_Param.vidInput.errormsg.push_back("- INPUT_VIDEO_PATH : " + *tok_iter + " not exists.");
            e = true;
        }else{
            m_Param.vidInput.INPUT_VIDEO_PATH.push_back(*tok_iter);
        }
    }

    if(!e) m_Param.vidInput.status = true;

}

void freeture::CfgParam::loadCamParam() {

    bool e = false;
    int camera_id;
    freeture::Device* device = manager.getDevice();
    if(m_Param.DEVICE_ID == -1) {

        loadDeviceID();
        loadCameraSerial();
        loadCameraInit();
        if(m_Param.DEVICE_ID == -1) {
            return;
        }
    }

    camera_id = m_Param.DEVICE_ID;
    int camera_id_f = manager.getCameraDeviceBySerial(m_Param.CAMERA_SERIAL);
    if(camera_id_f == -2)
    {
        freeture::Log("CAMERA WITH SERIAL " + m_Param.CAMERA_SERIAL + " NOT FOUND");
        return;
    }
    if(camera_id_f != -1) camera_id = camera_id_f;
    
    if(!device->createCamera(camera_id, true)) {
        freeture::Log("FAIL CREATING CAMERA ");
        return;
    }

    if(m_Param.CAMERA_INIT)
    {
        if(!device->firstIinitializeCamera(m_Param.CAMERA_INIT_CONFIG))
        {
            freeture::Log("Inizializing camera using " + m_Param.CAMERA_INIT_CONFIG + " Failed!");
            return;
        }
        
        
    }


    if(!m_Cfg.Get("ACQ_FPS", m_Param.camInput.ACQ_FPS)) {
        m_Param.camInput.errormsg.push_back("- ACQ_FPS : Fail to get value.");
        e = true;
    }

    //-------------------------------------------------------------------

    string pixfmt;
    if(!m_Cfg.Get("ACQ_FORMAT", pixfmt)) {
        m_Param.camInput.errormsg.push_back("- ACQ_FORMAT : Fail to get value.");
        e = true;
    }else {
        try {
            EParser<CamPixFmt> camPixFmt;
            m_Param.camInput.ACQ_FORMAT = camPixFmt.parseEnum("ACQ_FORMAT", pixfmt);
        }catch (std::exception &ex) {
            m_Param.camInput.errormsg.push_back("- ACQ_FORMAT : " + string(ex.what()));
            e = true;
        }
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("ACQ_RES_CUSTOM_SIZE", m_Param.camInput.ACQ_RES_CUSTOM_SIZE)) {
        m_Param.camInput.errormsg.push_back("- ACQ_RES_CUSTOM_SIZE : Fail to get value.");
        e = true;
    }else{
        
        if(m_Param.camInput.ACQ_RES_CUSTOM_SIZE) {
            string acq_offset;
            if(!m_Cfg.Get("ACQ_OFFSET", acq_offset)) {
                m_Param.camInput.errormsg.push_back("- ACQ_OFFSET : Fail to get value.");
                e = true;
            }else {

                if(acq_offset.find(",") != std::string::npos) {
                    //std::cout << "++++++++++++++++++++++++++++++++++++++ FIND ACQ OFFSET " << acq_offset << std::endl;
                    string offsetx = acq_offset.substr(0,acq_offset.find(","));
                    string offsety = acq_offset.substr(acq_offset.find(",")+1,string::npos);
                    int mStartX = atoi(offsetx.c_str());
                    int mStartY = atoi(offsety.c_str());

                    if(mStartX <= 0) {
                        m_Param.camInput.errormsg.push_back("- ACQ_OFFSET : X offset value is not correct.");
                        e = true;
                    }else{
                        m_Param.camInput.ACQ_STARTX = mStartX;
                        
                    }

                    if(mStartY <= 0) {
                        m_Param.camInput.errormsg.push_back("- ACQ_OFFSET : Y offset value is not correct.");
                        e = true;
                    }else{
                        m_Param.camInput.ACQ_STARTY = mStartY;
                    }

                }else {
                    m_Param.camInput.errormsg.push_back("- ACQ_OFFSET : Format is not correct. It must be : X,Y.");
                    e = true;
                }
            }

            string acq_res_custome_size;
            if(!m_Cfg.Get("ACQ_RES_SIZE", acq_res_custome_size)) {
                m_Param.camInput.errormsg.push_back("- ACQ_RES_SIZE : Fail to get value.");
                e = true;
            }else {

                if(acq_res_custome_size.find("x") != std::string::npos) {
                    //std::cout << "++++++++++++++++++++++++++++++++++++++ FIND ACQ RES SIZE " << acq_res_custome_size << std::endl;
                    string width = acq_res_custome_size.substr(0,acq_res_custome_size.find("x"));
                    string height = acq_res_custome_size.substr(acq_res_custome_size.find("x")+1,string::npos);
                    int mSizeWidth = atoi(width.c_str());
                    int mSizeHeight = atoi(height.c_str());

                    if(mSizeHeight <= 0) {
                        m_Param.camInput.errormsg.push_back("- ACQ_RES_SIZE : Height value is not correct.");
                        e = true;
                    }else{
                        //std::cout << "++++++++++++++++++++++++++++++++++++++ ACQ_HEIGHT " << mSizeHeight << std::endl;
                        m_Param.camInput.ACQ_HEIGHT = mSizeHeight;
                    }

                    if(mSizeWidth <= 0) {
                        m_Param.camInput.errormsg.push_back("- ACQ_RES_SIZE : Width value is not correct.");
                        e = true;
                    }else{
                        m_Param.camInput.ACQ_WIDTH = mSizeWidth;
                    }

                }else {
                    m_Param.camInput.errormsg.push_back("- ACQ_RES_SIZE : Format is not correct. It must be : WxH.");
                    e = true;
                }
            }
        }else {

            m_Param.camInput.ACQ_STARTX = 0;
            m_Param.camInput.ACQ_STARTY = 0;
            m_Param.camInput.ACQ_HEIGHT = 1080;
            m_Param.camInput.ACQ_WIDTH = 1440;

        }
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("SHIFT_BITS", m_Param.camInput.SHIFT_BITS)) {
        m_Param.camInput.errormsg.push_back("- SHIFT_BITS : Fail to get value.");
        e = true;
    }

    double ming= -1, maxg = -1;
    double mine = -1, maxe = -1;
    double minf = -1, maxf = -1;
    device->getCameraFPSBounds(minf, maxf);
    device->setCameraDayGain();
    device->getCameraGainBounds(ming, maxg);
    device->getCameraExposureBounds(mine, maxe);


    //-------------------------------------------------------------------

    if(!m_Cfg.Get("ACQ_NIGHT_EXPOSURE", m_Param.camInput.ACQ_NIGHT_EXPOSURE)) {
        m_Param.camInput.errormsg.push_back("- ACQ_NIGHT_EXPOSURE : Fail to get value.");
        e = true;
    }else{

        if(mine != -1 && maxe != -1) {
            if(m_Param.camInput.ACQ_NIGHT_EXPOSURE < mine || m_Param.camInput.ACQ_NIGHT_EXPOSURE > maxe) {
                m_Param.camInput.errormsg.push_back("- ACQ_NIGHT_EXPOSURE : Value <" +
                    Conversion::intToString(m_Param.camInput.ACQ_NIGHT_EXPOSURE) +
                    "> is not correct. \nAvailable range is from " +
                    Conversion::intToString(mine) + " to " +
                    Conversion::intToString(maxe));
                e = true;
            }
        }

        std::cout << "EXP NIGHT " << m_Param.camInput.ACQ_NIGHT_EXPOSURE << std::endl;
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("ACQ_NIGHT_GAIN", m_Param.camInput.ACQ_NIGHT_GAIN)) {
        m_Param.camInput.errormsg.push_back("- ACQ_NIGHT_GAIN : Fail to get value.");
        e = true;
    }else{

        if(ming != -1 && maxg != -1) {
            if(m_Param.camInput.ACQ_NIGHT_GAIN < ming || m_Param.camInput.ACQ_NIGHT_GAIN > maxg) {
                m_Param.camInput.errormsg.push_back("- ACQ_NIGHT_GAIN : Value <" +
                    Conversion::intToString(m_Param.camInput.ACQ_NIGHT_GAIN) +
                    "> is not correct. \nAvailable range is from " +
                    Conversion::intToString(ming) + " to " +
                    Conversion::intToString(maxg));
                e = true;
            }
        }
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("ACQ_DAY_EXPOSURE", m_Param.camInput.ACQ_DAY_EXPOSURE)) {
        m_Param.camInput.errormsg.push_back("- ACQ_DAY_EXPOSURE : Fail to get value.");
        e = true;
    }else{

        if(mine != -1 && maxe != -1) {
            if(m_Param.camInput.ACQ_DAY_EXPOSURE < mine || m_Param.camInput.ACQ_DAY_EXPOSURE > maxe) {
                m_Param.camInput.errormsg.push_back("- ACQ_DAY_EXPOSURE : Value <" +
                    Conversion::intToString(m_Param.camInput.ACQ_DAY_EXPOSURE) +
                    "> is not correct. \nAvailable range is from " +
                    Conversion::intToString(mine) + " to " +
                    Conversion::intToString(maxe));
                e = true;
            }
        }
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("ACQ_DAY_GAIN", m_Param.camInput.ACQ_DAY_GAIN)) {
        m_Param.camInput.errormsg.push_back("- ACQ_DAY_GAIN : Fail to get value.");
        e = true;
    }else{

        if(ming != -1 && maxg != -1) {
            if(m_Param.camInput.ACQ_DAY_GAIN < ming || m_Param.camInput.ACQ_DAY_GAIN > maxg) {
                m_Param.camInput.errormsg.push_back("- ACQ_DAY_GAIN : Value <" +
                    Conversion::intToString(m_Param.camInput.ACQ_DAY_GAIN) +
                    "> is not correct. \nAvailable range is from " +
                    Conversion::intToString(ming) + " to " +
                    Conversion::intToString(maxg));
                e = true;
            }
        }
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("EXPOSURE_CONTROL_FREQUENCY", m_Param.camInput.EXPOSURE_CONTROL_FREQUENCY)) {
        m_Param.camInput.errormsg.push_back("- EXPOSURE_CONTROL_FREQUENCY : Fail to get value.");
        e = true;
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("EXPOSURE_CONTROL_SAVE_IMAGE", m_Param.camInput.EXPOSURE_CONTROL_SAVE_IMAGE)) {
        m_Param.camInput.errormsg.push_back("- EXPOSURE_CONTROL_SAVE_IMAGE : Fail to get value.");
        e = true;
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("EXPOSURE_CONTROL_SAVE_INFOS", m_Param.camInput.EXPOSURE_CONTROL_SAVE_INFOS)) {
        m_Param.camInput.errormsg.push_back("- EXPOSURE_CONTROL_SAVE_INFOS : Fail to get value.");
        e = true;
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("EPHEMERIS_ENABLED", m_Param.camInput.ephem.EPHEMERIS_ENABLED)) {
        m_Param.camInput.errormsg.push_back("- EPHEMERIS_ENABLED : Fail to get value.");
        e = true;
    }

    //-------------------------------------------------------------------
    if(m_Param.camInput.ephem.EPHEMERIS_ENABLED) {
        if(!m_Cfg.Get("SUN_HORIZON_1", m_Param.camInput.ephem.SUN_HORIZON_1)) {
            m_Param.camInput.errormsg.push_back("- SUN_HORIZON_1 : Fail to get value.");
            e = true;
        }

        //-------------------------------------------------------------------

        if(!m_Cfg.Get("SUN_HORIZON_2", m_Param.camInput.ephem.SUN_HORIZON_2)) {
            m_Param.camInput.errormsg.push_back("- SUN_HORIZON_2 : Fail to get value.");
            e = true;
        }
    }

    //-------------------------------------------------------------------

    if(!m_Param.camInput.ephem.EPHEMERIS_ENABLED) {

        string sunrise_time;
        if(!m_Cfg.Get("SUNRISE_TIME", sunrise_time)) {
            m_Param.camInput.errormsg.push_back("- SUNRISE_TIME : Fail to get value.");
            e = true;
        }else{

            if(sunrise_time.find(":") != std::string::npos) {

                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                boost::char_separator<char> sep(":");
                tokenizer tokens(sunrise_time, sep);

                m_Param.camInput.ephem.SUNRISE_TIME.clear();
                for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter){
                    m_Param.camInput.ephem.SUNRISE_TIME.push_back(atoi((*tok_iter).c_str()));
                }

                if(m_Param.camInput.ephem.SUNRISE_TIME.size() == 2) {
                    if(m_Param.camInput.ephem.SUNRISE_TIME.at(0) < 0 || m_Param.camInput.ephem.SUNRISE_TIME.at(0) >= 24) {
                        m_Param.camInput.errormsg.push_back("- SUNRISE_TIME : Hours value must be between 0 - 23");
                        e = true;
                    }

                    if(m_Param.camInput.ephem.SUNRISE_TIME.at(1) < 0 || m_Param.camInput.ephem.SUNRISE_TIME.at(0) >= 60) {
                        m_Param.camInput.errormsg.push_back("- SUNRISE_TIME : Minutes value must be between 0 - 59");
                        e = true;
                    }
                }

            }else {
                m_Param.camInput.errormsg.push_back("- SUNRISE_TIME : Format is not correct. It must be : HH:MM");
                e = true;
            }
        }

        //-------------------------------------------------------------------

        string sunset_time;
        if(!m_Cfg.Get("SUNSET_TIME", sunset_time)) {
            m_Param.camInput.errormsg.push_back("- SUNSET_TIME : Fail to get value.");
            e = true;
        }else{

            if(sunset_time.find(":") != std::string::npos) {

                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                boost::char_separator<char> sep(":");
                tokenizer tokens(sunset_time, sep);

                m_Param.camInput.ephem.SUNSET_TIME.clear();
                for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter)
                    m_Param.camInput.ephem.SUNSET_TIME.push_back(atoi((*tok_iter).c_str()));

                if(m_Param.camInput.ephem.SUNSET_TIME.size() == 2) {
                    if(m_Param.camInput.ephem.SUNSET_TIME.at(0) < 0 || m_Param.camInput.ephem.SUNSET_TIME.at(0) >= 24) {
                        m_Param.camInput.errormsg.push_back("- SUNSET_TIME : Hours value must be between 0 - 23");
                        e = true;
                    }

                    if(m_Param.camInput.ephem.SUNSET_TIME.at(1) < 0 || m_Param.camInput.ephem.SUNSET_TIME.at(0) >= 60) {
                        m_Param.camInput.errormsg.push_back("- SUNSET_TIME : Minutes value must be between 0 - 59");
                        e = true;
                    }
                }

            }else {
                m_Param.camInput.errormsg.push_back("- SUNSET_TIME : Format is not correct. It must be : HH:MM");
                e = true;
            }
        }

        //-------------------------------------------------------------------

        if(!m_Cfg.Get("SUNSET_DURATION", m_Param.camInput.ephem.SUNSET_DURATION)) {
            m_Param.camInput.errormsg.push_back("- SUNSET_DURATION : Fail to get value.");
            e = true;
        }

        //-------------------------------------------------------------------

        if(!m_Cfg.Get("SUNRISE_DURATION", m_Param.camInput.ephem.SUNRISE_DURATION)) {
            m_Param.camInput.errormsg.push_back("- SUNRISE_DURATION : Fail to get value.");
            e = true;
        }
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("ACQ_REGULAR_ENABLED", m_Param.camInput.regcap.ACQ_REGULAR_ENABLED)) {
        m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_ENABLED : Fail to get value.");
        e = true;
    }else {
        if(m_Param.camInput.regcap.ACQ_REGULAR_ENABLED) {
            //-------------------------------------------------------------------

            string reg_mode;
            if(!m_Cfg.Get("ACQ_REGULAR_MODE", reg_mode)) {
                e = true;
                m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_MODE : Fail to load value.");
            }else {
                try {
                    EParser<TimeMode> regMode;
                    m_Param.camInput.regcap.ACQ_REGULAR_MODE = regMode.parseEnum("ACQ_REGULAR_MODE", reg_mode);
                }catch (std::exception &ex) {
                    e = true;
                    m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_MODE : " + string(ex.what()));
                }
            }

            //-------------------------------------------------------------------

            {

                string img_prefix;
                if(!m_Cfg.Get("ACQ_REGULAR_PRFX", img_prefix)) {
                    e = true;
                    m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_PRFX : Fail to load value.");
                }else {
                    m_Param.camInput.regcap.ACQ_REGULAR_PRFX = img_prefix;
                }
            }

            //-------------------------------------------------------------------

            {

                string img_output;
                if(!m_Cfg.Get("ACQ_REGULAR_OUTPUT", img_output)) {
                    e = true;
                    m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_OUTPUT : Fail to load value.");
                }else {
                    try {
                        EParser<ImgFormat> imgOutput;
                        m_Param.camInput.regcap.ACQ_REGULAR_OUTPUT = imgOutput.parseEnum("ACQ_REGULAR_OUTPUT", img_output);
                    }catch (std::exception &ex) {
                        e = true;
                        m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_OUTPUT : " + string(ex.what()));
                    }
                }
            }

            //-------------------------------------------------------------------

            string regAcqParam;
            if(!m_Cfg.Get("ACQ_REGULAR_CFG", regAcqParam)) {
                e = true;
                m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_CFG : Fail to load value.");
            }else {
                std::transform(regAcqParam.begin(), regAcqParam.end(),regAcqParam.begin(), ::toupper);

                typedef boost::tokenizer<boost::char_separator<char> > tokenizer1;
                boost::char_separator<char> sep1("HMSEGFN");
                tokenizer1 tokens1(regAcqParam, sep1);

                vector<string> res1;
                for(tokenizer1::iterator tokIter = tokens1.begin();tokIter != tokens1.end(); ++tokIter)
                    res1.push_back(*tokIter);

                if(res1.size() == 7) {

                    // Get regular acquisition time interval.
                    if(atoi(res1.at(0).c_str())< 0 || atoi(res1.at(0).c_str()) >= 24) {
                        e = true;
                        m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_CFG : Hours can't have the value <" + res1.at(0) + ">.\nAvailable range is from 0 to 23.");
                    }

                    if(atoi(res1.at(1).c_str())< 0 || atoi(res1.at(1).c_str()) >= 60) {
                        e = true;
                        m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_CFG : Minutes can't have the value <" + res1.at(1) + ">.\nAvailable range is from 0 to 23.");
                    }

                    if(atoi(res1.at(2).c_str())< 0 || atoi(res1.at(2).c_str()) >= 60) {
                        e = true;
                        m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_CFG : Seconds can't have the value <" + res1.at(2) + ">.\nAvailable range is from 0 to 23.");
                    }

                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.interval = atoi(res1.at(0).c_str()) * 3600 + atoi(res1.at(1).c_str()) * 60 + atoi(res1.at(2).c_str());

                    // Get regular acquisition exposure time.
                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.exp = atoi(res1.at(3).c_str());

                    if(mine != -1 && maxe != -1) {
                        if(m_Param.camInput.regcap.ACQ_REGULAR_CFG.exp < mine || m_Param.camInput.regcap.ACQ_REGULAR_CFG.exp > maxe) {
                            m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_CFG : Exposure value <" +
                                Conversion::intToString(m_Param.camInput.regcap.ACQ_REGULAR_CFG.exp) +
                                "> is not correct. \nAvailable range is from " +
                                Conversion::intToString(mine) + " to " +
                                Conversion::intToString(maxe));
                            e = true;
                        }
                    }

                    // Get regular acquisition gain.
                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.gain = atoi(res1.at(4).c_str());

                    if(ming != -1 && maxg != -1) {
                        if(m_Param.camInput.regcap.ACQ_REGULAR_CFG.gain < ming || m_Param.camInput.regcap.ACQ_REGULAR_CFG.gain > maxg) {
                            m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_CFG : Gain value <" +
                                Conversion::intToString(m_Param.camInput.regcap.ACQ_REGULAR_CFG.gain) +
                                "> is not correct. \nAvailable range is from " +
                                Conversion::intToString(ming) + " to " +
                                Conversion::intToString(maxg));
                            e = true;
                        }
                    }

                    // Get regular acquisition repetition.
                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.rep = atoi(res1.at(6).c_str());

                    // Get regular acquisition format.
                    m_Param.camInput.regcap.ACQ_REGULAR_CFG.fmt = static_cast<CamPixFmt>(atoi(res1.at(5).c_str()));
                    EParser<CamPixFmt> fmt;
                    if(fmt.getStringEnum(m_Param.camInput.regcap.ACQ_REGULAR_CFG.fmt) == ""){
                        e = true;
                        m_Param.camInput.errormsg.push_back("- ACQ_REGULAR_CFG : Fail to extract pixel format on " + regAcqParam + ". Check if index <" + res1.at(5) + "> exits.");
                    }
                }
            }
        }
    }

    //-------------------------------------------------------------------

    if(!m_Cfg.Get("ACQ_SCHEDULE_ENABLED", m_Param.camInput.schcap.ACQ_SCHEDULE_ENABLED)) {
        m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE_ENABLED : Fail to get value.");
        e = true;
    }else{

        if(m_Param.camInput.schcap.ACQ_SCHEDULE_ENABLED) {

            if(!m_Param.camInput.regcap.ACQ_REGULAR_ENABLED) {
                //-------------------------------------------------------------------

                {
                    string img_output;
                    if(!m_Cfg.Get("ACQ_SCHEDULE_OUTPUT", img_output)) {
                        e = true;
                        m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE_OUTPUT : Fail to load value.");
                    }else {
                        try {
                            EParser<ImgFormat> imgOutput;
                            m_Param.camInput.schcap.ACQ_SCHEDULE_OUTPUT = imgOutput.parseEnum("ACQ_SCHEDULE_OUTPUT", img_output);
                        }catch (std::exception &ex) {
                            e = true;
                            m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE_OUTPUT : " + string(ex.what()));
                        }
                    }
                }

                //-------------------------------------------------------------------

                {
                    string sACQ_SCHEDULE;
                    if(!m_Cfg.Get("ACQ_SCHEDULE", sACQ_SCHEDULE)) {
                        e = true;
                        m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : Fail to load value.");
                    }else {

                        vector<string> sch1;

                        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                        boost::char_separator<char> sep(",");
                        tokenizer tokens(sACQ_SCHEDULE, sep);

                        for(tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter) {
                            string s = *tok_iter;
                            std::transform(s.begin(), s.end(),s.begin(), ::toupper);
                            sch1.push_back(s);
                        }

                        for(int i = 0; i < sch1.size(); i++) {

                            typedef boost::tokenizer<boost::char_separator<char> > tokenizer_;
                            boost::char_separator<char> sep_("HMSEGFN");
                            tokenizer tokens_(sch1.at(i), sep_);

                            vector<string> sp;

                            for(tokenizer::iterator tok_iter_ = tokens_.begin();tok_iter_ != tokens_.end(); ++tok_iter_)
                                sp.push_back(*tok_iter_);

                            if(sp.size() == 7) {

                                scheduleParam spa;
                                bool status = true;
                                spa.hours = atoi(sp.at(0).c_str());
                                if(spa.hours< 0 || spa.hours >= 24) {
                                    e = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : In " + sch1.at(i) + ". Hours can't have the value <" + Conversion::intToString(spa.hours) + ">.\nAvailable range is from 0 to 23.");
                                }
                                spa.min = atoi(sp.at(1).c_str());
                                if(spa.min< 0 || spa.min >= 60) {
                                    e = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : In " + sch1.at(i) + ". Minutes can't have the value <" + Conversion::intToString(spa.min) + ">.\nAvailable range is from 0 to 59.");
                                }
                                spa.sec = atoi(sp.at(2).c_str());
                                if(spa.sec< 0 || spa.sec >= 60) {
                                    e = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : In " + sch1.at(i) + ". Seconds can't have the value <" + Conversion::intToString(spa.sec) + ">.\nAvailable range is from 0 to 59.");
                                }
                                spa.exp = atoi(sp.at(3).c_str());

                                if(mine != -1 && maxe != -1) {
                                    if(spa.exp < mine || spa.exp > maxe) {
                                        m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : In " + sch1.at(i) + ". Exposure value <" +
                                            Conversion::intToString(spa.exp) +
                                            "> is not correct. \nAvailable range is from " +
                                            Conversion::intToString(mine) + " to " +
                                            Conversion::intToString(maxe));
                                        e = true;
                                        status = false;
                                    }
                                }

                                spa.gain = atoi(sp.at(4).c_str());

                                if(ming != -1 && maxg != -1) {
                                    if(spa.gain < ming || spa.gain > maxg) {
                                        m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : In " + sch1.at(i) + ". Gain value <" +
                                            Conversion::intToString(spa.gain) +
                                            "> is not correct. \nAvailable range is from " +
                                            Conversion::intToString(ming) + " to " +
                                            Conversion::intToString(maxg));
                                        e = true;
                                        status = false;
                                    }
                                }

                                spa.rep = atoi(sp.at(6).c_str());
                                if(spa.rep< 0 || spa.rep >= 60) {
                                    e = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : One repetition must be defined at least.");
                                }
                                spa.fmt = static_cast<CamPixFmt>(atoi(sp.at(5).c_str()));
                                EParser<CamPixFmt> fmt;
                                if(fmt.getStringEnum(spa.fmt) == ""){
                                    e = true;
                                    status = false;
                                    m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : Fail to extract pixel format for :  " + sch1.at(i) + ". Index <" + sp.at(5) + "> not exist.");
                                }

                                if(status)
                                    m_Param.camInput.schcap.ACQ_SCHEDULE.push_back(spa);

                            }
                        }

                        // Order scheduled acquisition

                        if(m_Param.camInput.schcap.ACQ_SCHEDULE.size() != 0){

                            // Sort time in list.
                            vector<scheduleParam> tempSchedule;

                            do{

                                int minH; int minM; int minS; bool init = false;

                                vector<scheduleParam>::iterator it;
                                vector<scheduleParam>::iterator it_select;

                                for(it = m_Param.camInput.schcap.ACQ_SCHEDULE.begin(); it != m_Param.camInput.schcap.ACQ_SCHEDULE.end(); ++it){

                                    if(!init){

                                        minH = (*it).hours;
                                        minM = (*it).min;
                                        minS = (*it).sec;
                                        it_select = it;
                                        init = true;

                                    }else{

                                        if((*it).hours < minH){

                                            minH = (*it).hours;
                                            minM = (*it).min;
                                            minS = (*it).sec;
                                            it_select = it;

                                        }else if((*it).hours == minH){

                                            if((*it).min < minM){

                                                minH = (*it).hours;
                                                minM = (*it).min;
                                                minS = (*it).sec;
                                                it_select = it;

                                            }else if((*it).min == minM){

                                                if((*it).sec < minS){

                                                    minH = (*it).hours;
                                                    minM = (*it).min;
                                                    minS = (*it).sec;
                                                    it_select = it;

                                                }

                                            }

                                        }

                                    }

                                }

                                if(init){

                                    tempSchedule.push_back((*it_select));
                                    //cout << "-> " << (*it_select).hours << "H " << (*it_select).min << "M " << (*it_select).sec << "S " << endl;
                                    m_Param.camInput.schcap.ACQ_SCHEDULE.erase(it_select);

                                }

                            }while(m_Param.camInput.schcap.ACQ_SCHEDULE.size() != 0);

                            m_Param.camInput.schcap.ACQ_SCHEDULE = tempSchedule;

                        }
                    }
                }

            }else{
                e = true;
                m_Param.camInput.errormsg.push_back("- ACQ_SCHEDULE : Disable ACQ_REGULAR_ENABLED to use ACQ_SCHEDULE.");
            }
        }
    }

    if(!e) m_Param.camInput.status = true;
}

void freeture::CfgParam::loadDetParam() {

    bool e = false;

    if(!m_Cfg.Get("ACQ_BUFFER_SIZE", m_Param.det.ACQ_BUFFER_SIZE)) {
        e = true;
        m_Param.det.errormsg.push_back("- ACQ_BUFFER_SIZE : Fail to load value.");
    }

    if(!m_Cfg.Get("ACQ_MASK_ENABLED", m_Param.det.ACQ_MASK_ENABLED)) {
        e = true;
        m_Param.det.errormsg.push_back("- ACQ_MASK_ENABLED : Fail to load value.");
    }else{

        if(m_Param.det.ACQ_MASK_ENABLED) {

            if(!m_Cfg.Get("ACQ_MASK_PATH", m_Param.det.ACQ_MASK_PATH)) {
                e = true;
                m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Fail to load value.");
            }else {
                Mat tempmask = imread(m_Param.det.ACQ_MASK_PATH, IMREAD_GRAYSCALE);

                if(!tempmask.data) {
                    e = true;
                    m_Param.det.errormsg.push_back("- MASK : Fail to load the mask image. No data.");
                    // Add test to compare mask size to a capture from camera or video or frame file
                }else {

                    tempmask.copyTo(m_Param.det.MASK);

                    if(m_Param.DEVICE_ID = -1) {
                        mDevice->setVerbose(false);
                        mDevice->listDevices(false);
                        m_InputType = mDevice->getDeviceType(mDevice->getDeviceSdk(m_Param.DEVICE_ID));

                        switch(m_InputType) {

                            case VIDEO :
                                {
                                    if(vidParamIsCorrect()) {

                                        for(int i = 0; i < m_Param.vidInput.INPUT_VIDEO_PATH.size(); i++) {
                                            VideoCapture cap = VideoCapture(m_Param.vidInput.INPUT_VIDEO_PATH.at(i));
                                            if(cap.isOpened()) {
                                                if(cap.get(cv::CAP_PROP_FRAME_HEIGHT) != tempmask.rows) {
                                                    e = true;
                                                    m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Mask's height (" +
                                                        Conversion::intToString(tempmask.rows) +
                                                        ") is not correct with " + m_Param.vidInput.INPUT_VIDEO_PATH.at(i) + " (" +
                                                        Conversion::intToString(cap.get(cv::CAP_PROP_FRAME_HEIGHT)) + ")");
                                                }

                                                if(cap.get(cv::CAP_PROP_FRAME_WIDTH) != tempmask.cols) {
                                                    e = true;
                                                    m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Mask's width (" +
                                                        Conversion::intToString(tempmask.cols) +
                                                        ") is not correct with " + m_Param.vidInput.INPUT_VIDEO_PATH.at(i) + " (" +
                                                        Conversion::intToString(cap.get(cv::CAP_PROP_FRAME_WIDTH)) + ")");
                                                }
                                            }else{
                                                e = true;
                                                m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Check mask's size. Fail to open " + m_Param.vidInput.INPUT_VIDEO_PATH.at(i));
                                            }
                                        }

                                    }else{
                                        e = true;
                                        m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Check mask's size. Video parameters loading failed.");
                                    }
                                }
                                break;

                            case SINGLE_FITS_FRAME :

                                {
                                    if(framesParamIsCorrect()) {
                                        for(int i = 0; i < m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.size(); i++) {
                                            // Search a fits file.
                                            bool fitsfilefound = false;
                                            string filefound = "";
                                            path p(m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.at(i));
                                            for(directory_iterator file(p);file!= directory_iterator(); ++file){
                                                path curr(file->path());
                                                if(is_regular_file(curr)) {
                                                    if(file->path().string().find(".fit") != std::string::npos) {
                                                        fitsfilefound = true;
                                                        filefound = file->path().string();
                                                        break;
                                                    }
                                                }
                                            }

                                            if(fitsfilefound) {
                                                Fits2D f(filefound);
                                                int h = 0,w = 0;

                                                if(!f.readIntKeyword("NAXIS1", w)){
                                                    e = true;
                                                    m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Check mask's size. Fail to read NAXIS1. " + m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.at(i));
                                                }

                                                if(!f.readIntKeyword("NAXIS2", h)){
                                                    e = true;
                                                    m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Check mask's size. Fail to read NAXIS2. " + m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.at(i));
                                                }

                                                if(h!=0 && w!=0) {

                                                    if(h != tempmask.rows) {
                                                        e = true;
                                                        m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Mask's height (" +
                                                            Conversion::intToString(tempmask.rows) +
                                                            ") is not correct with " + m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.at(i) + " (" +
                                                            Conversion::intToString(h) + ")");
                                                    }

                                                    if(w != tempmask.cols) {
                                                        e = true;
                                                        m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Mask's width (" +
                                                            Conversion::intToString(tempmask.cols) +
                                                            ") is not correct with " + m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.at(i) + " (" +
                                                            Conversion::intToString(w) + ")");
                                                    }
                                                }

                                            }else{
                                                e = true;
                                                m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Check mask's size. No fits file found in " + m_Param.framesInput.INPUT_FRAMES_DIRECTORY_PATH.at(i));
                                            }
                                        }
                                    }
                                }

                                break;

                            case CAMERA :
                                {
                                    /*if(camParamIsCorrect()) {

                                    }*/
                                }
                                break;

                            default :
                                e = true;
                                m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Fail to create device to check mask's size.");

                        }



                    }else {
                        e = true;
                        m_Param.det.errormsg.push_back("- ACQ_MASK_PATH : Fail to create device to check mask's size. CAMERA_ID not loaded.");
                    }
                }
            }
        }
    }

    if(!m_Cfg.Get("DET_ENABLED", m_Param.det.DET_ENABLED)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_ENABLED : Fail to load value.");
    }

    string det_mode;
    if(!m_Cfg.Get("DET_MODE", det_mode)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_MODE : Fail to load value.");
    }else {
        try {
            EParser<TimeMode> detMode;
            m_Param.det.DET_MODE = detMode.parseEnum("DET_MODE", det_mode);
        }catch (std::exception &ex) {
            e = true;
            m_Param.det.errormsg.push_back("- DET_MODE : " + string(ex.what()));
        }
    }

    if(!m_Cfg.Get("DET_DEBUG", m_Param.det.DET_DEBUG)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_DEBUG : Fail to load value.");
    }else{

        if(m_Param.det.DET_DEBUG){

            if(!m_Cfg.Get("DET_DEBUG_PATH", m_Param.det.DET_DEBUG_PATH)) {
                e = true;
                m_Param.det.errormsg.push_back("- DET_DEBUG_PATH : Fail to load value.");
            }else{

                namespace fs = boost::filesystem;
                path p(m_Param.det.DET_DEBUG_PATH);

                if(!fs::exists(p)){
                    if(!fs::create_directory(p)){
                        e = true;
                        m_Param.det.errormsg.push_back("- DET_DEBUG_PATH : Can't create Debug Path.");
                    }
                }
            }
        }
    }

    if(!m_Cfg.Get("DET_TIME_AROUND", m_Param.det.DET_TIME_AROUND)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_TIME_AROUND : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_TIME_MAX", m_Param.det.DET_TIME_MAX)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_TIME_MAX : Fail to load value.");
    }else{

        // If input device type is frames or video, increase DET_TIME_MAX because
        // time can not be take account as the time interval between can be increased.
        if(m_InputType == VIDEO || m_InputType == SINGLE_FITS_FRAME) {
            m_Param.det.DET_TIME_MAX = 10000;
        }else{
            if(m_Param.det.DET_TIME_MAX <= 0 || m_Param.det.DET_TIME_MAX > 30) {
                e = true;
                m_Param.det.errormsg.push_back("- DET_TIME_MAX : Available range is from 1 to 30 seconds.");
            }
        }
    }

    string det_mthd;
    if(!m_Cfg.Get("DET_METHOD", det_mthd)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_METHOD : Fail to load value.");
    }else {
        try {
            EParser<DetMeth> detMthd;
            m_Param.det.DET_METHOD = detMthd.parseEnum("DET_METHOD", det_mthd);
        }catch (std::exception &ex) {
            e = true;
            m_Param.st.errormsg.push_back("- DET_METHOD : " + string(ex.what()));
        }
    }

    if(!m_Cfg.Get("DET_SAVE_FITS3D", m_Param.det.DET_SAVE_FITS3D)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_FITS3D : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_SAVE_FITS2D", m_Param.det.DET_SAVE_FITS2D)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_FITS2D : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_SAVE_SUM", m_Param.det.DET_SAVE_SUM)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_SUM : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_SUM_REDUCTION", m_Param.det.DET_SUM_REDUCTION)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SUM_REDUCTION : Fail to load value.");
    }

    string det_sum_mthd;
    if(!m_Cfg.Get("DET_SUM_MTHD", det_sum_mthd)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SUM_MTHD : Fail to load value.");
    }else {
        try {
            EParser<StackMeth> detSumMthd;
            m_Param.det.DET_SUM_MTHD = detSumMthd.parseEnum("DET_SUM_MTHD", det_sum_mthd);
        }catch (std::exception &ex) {
            e = true;
            m_Param.det.errormsg.push_back("- DET_SUM_MTHD : " + string(ex.what()));
        }
    }

    if(!m_Cfg.Get("DET_SAVE_SUM_WITH_HIST_EQUALIZATION", m_Param.det.DET_SAVE_SUM_WITH_HIST_EQUALIZATION)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_SUM_WITH_HIST_EQUALIZATION : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_SAVE_AVI", m_Param.det.DET_SAVE_AVI)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_AVI : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_UPDATE_MASK", m_Param.det.DET_UPDATE_MASK)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_UPDATE_MASK : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_UPDATE_MASK_FREQUENCY", m_Param.det.DET_UPDATE_MASK_FREQUENCY)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_UPDATE_MASK_FREQUENCY : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_DEBUG_UPDATE_MASK", m_Param.det.DET_DEBUG_UPDATE_MASK)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_DEBUG_UPDATE_MASK : Fail to load value.");
    }else{

        if(m_Param.det.DET_DEBUG_UPDATE_MASK){

            if(!m_Cfg.Get("DET_DEBUG_PATH", m_Param.det.DET_DEBUG_PATH)) {
                e = true;
                m_Param.det.errormsg.push_back("- DET_DEBUG_PATH : Fail to load value.");
            }else{

                namespace fs = boost::filesystem;
                path p(m_Param.det.DET_DEBUG_PATH);

                if(!fs::exists(p)){
                    try {
                        fs::create_directory(p);
                    } catch (std::exception &ex) {
                        m_Param.det.errormsg.push_back("- DET_DEBUG_PATH : Can't create Debug Path. Debug Path must exist as DET_DEBUG_UPDATE_MASK is enabled.");
                        e = true;
                    }
                }
            }
        }
    }

    // --------------------------------------------------------------------------------------

    if(!m_Cfg.Get("DET_DOWNSAMPLE_ENABLED", m_Param.det.DET_DOWNSAMPLE_ENABLED)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_DOWNSAMPLE_ENABLED : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_SAVE_GEMAP", m_Param.det.temporal.DET_SAVE_GEMAP)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_GEMAP : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_SAVE_DIRMAP", m_Param.det.temporal.DET_SAVE_DIRMAP)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_DIRMAP : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_SAVE_POS", m_Param.det.temporal.DET_SAVE_POS)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_POS : Fail to load value.");
    }

    if(!m_Cfg.Get("DET_LE_MAX", m_Param.det.temporal.DET_LE_MAX)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_LE_MAX : Fail to load value.");
    }else{

        if(m_Param.det.temporal.DET_LE_MAX < 1 || m_Param.det.temporal.DET_LE_MAX > 10) {

            e = true;
            m_Param.det.errormsg.push_back("- DET_LE_MAX : Available range is from 1 to 10.");

        }

    }

    if(!m_Cfg.Get("DET_GE_MAX", m_Param.det.temporal.DET_GE_MAX)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_GE_MAX : Fail to load value.");
    }else{

        if(m_Param.det.temporal.DET_GE_MAX < 1 || m_Param.det.temporal.DET_GE_MAX > 10) {

            e = true;
            m_Param.det.errormsg.push_back("- DET_GE_MAX : Available range is from 1 to 10.");

        }

    }

    /*if(!m_Cfg.Get("DET_SAVE_GE_INFOS", m_Param.det.temporal.DET_SAVE_GE_INFOS)) {
        e = true;
        m_Param.det.errormsg.push_back("- DET_SAVE_GE_INFOS : Fail to load value.");
    }*/

    if(!e) m_Param.det.status = true;

}

void freeture::CfgParam::loadStackParam() {

    bool e = false;

    if(!m_Cfg.Get("STACK_ENABLED", m_Param.st.STACK_ENABLED)) {
        e = true;
        m_Param.st.errormsg.push_back("- STACK_ENABLED : Fail to load value.");
    }

    string stack_mode;
    if(!m_Cfg.Get("STACK_MODE", stack_mode)) {
        e = true;
        m_Param.st.errormsg.push_back("- STACK_MODE : Fail to load value.");
    }else {
        try {
            EParser<TimeMode> stackMode;
            m_Param.st.STACK_MODE = stackMode.parseEnum("STACK_MODE", stack_mode);
        }catch (std::exception &ex) {
            e = true;
            m_Param.st.errormsg.push_back("- STACK_MODE : " + string(ex.what()));
        }
    }

    if(!m_Cfg.Get("STACK_TIME", m_Param.st.STACK_TIME)) {
        e = true;
        m_Param.st.errormsg.push_back("- STACK_TIME : Fail to load value.");
    }

    if(!m_Cfg.Get("STACK_INTERVAL", m_Param.st.STACK_INTERVAL)) {
        e = true;
        m_Param.st.errormsg.push_back("- STACK_INTERVAL : Fail to load value.");
    }

    string stack_mthd;
    if(!m_Cfg.Get("STACK_MTHD", stack_mthd)) {
        e = true;
        m_Param.st.errormsg.push_back("- STACK_MTHD : Fail to load value.");
    }else {
        try {
            EParser<StackMeth> stackMthd;
            m_Param.st.STACK_MTHD = stackMthd.parseEnum("STACK_MTHD", stack_mthd);
        }catch (std::exception &ex) {
            e = true;
            m_Param.st.errormsg.push_back("- STACK_MTHD : " + string(ex.what()));
        }
    }

    if(!m_Cfg.Get("STACK_REDUCTION", m_Param.st.STACK_REDUCTION)) {
        e = true;
        m_Param.st.errormsg.push_back("- STACK_REDUCTION : Fail to load value.");
    }

    if(!e) m_Param.st.status = true;

}

void freeture::CfgParam::loadStationParam() {

    bool e = false;

    if(!m_Cfg.Get("STATION_NAME", m_Param.station.STATION_NAME)) {
        e = true;
        m_Param.station.errormsg.push_back("- STATION_NAME : Fail to load value.");
    }

    if(!m_Cfg.Get("TELESCOP", m_Param.station.TELESCOP)) {
        e = true;
        m_Param.station.errormsg.push_back("- TELESCOP : Fail to load value.");
    }

    if(!m_Cfg.Get("OBSERVER", m_Param.station.OBSERVER)) {
        e = true;
        m_Param.station.errormsg.push_back("- OBSERVER : Fail to load value.");
    }

    if(!m_Cfg.Get("INSTRUME", m_Param.station.INSTRUME)) {
        e = true;
        m_Param.station.errormsg.push_back("- INSTRUME : Fail to load value.");
    }

    if(!m_Cfg.Get("CAMERA", m_Param.station.CAMERA)) {
        e = true;
        m_Param.station.errormsg.push_back("- CAMERA : Fail to load value.");
    }

    if(!m_Cfg.Get("FOCAL", m_Param.station.FOCAL)) {
        e = true;
        m_Param.station.errormsg.push_back("- FOCAL : Fail to load value.");
    }

    if(!m_Cfg.Get("APERTURE", m_Param.station.APERTURE)) {
        e = true;
        m_Param.station.errormsg.push_back("- APERTURE : Fail to load value.");
    }

    if(!m_Cfg.Get("SITELONG", m_Param.station.SITELONG)) {
        e = true;
        m_Param.station.errormsg.push_back("- SITELONG : Fail to load value.");
    }

    if(!m_Cfg.Get("SITELAT", m_Param.station.SITELAT)) {
        e = true;
        m_Param.station.errormsg.push_back("- SITELAT : Fail to load value.");
    }

    if(!m_Cfg.Get("SITEELEV", m_Param.station.SITEELEV)) {
        e = true;
        m_Param.station.errormsg.push_back("- SITEELEV : Fail to load value.");
    }

    if(!e) m_Param.station.status = true;
}

void freeture::CfgParam::loadFitskeysParam() {

    bool e = false;

    if(!m_Cfg.Get("K1", m_Param.fitskeys.K1)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- K1 : Fail to load value.");
    }

    if(!m_Cfg.Get("K2", m_Param.fitskeys.K2)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- K2 : Fail to load value.");
    }

    if(!m_Cfg.Get("FILTER", m_Param.fitskeys.FILTER)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- FILTER : Fail to load value.");
    }

    if(!m_Cfg.Get("CD1_1", m_Param.fitskeys.CD1_1)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- CD1_1 : Fail to load value.");
    }

    if(!m_Cfg.Get("CD1_2", m_Param.fitskeys.CD1_2)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- CD1_2 : Fail to load value.");
    }

    if(!m_Cfg.Get("CD2_1", m_Param.fitskeys.CD2_1)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- CD2_1 : Fail to load value.");
    }

    if(!m_Cfg.Get("CD2_2", m_Param.fitskeys.CD2_2)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- CD2_2 : Fail to load value.");
    }

    if(!m_Cfg.Get("XPIXEL", m_Param.fitskeys.XPIXEL)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- XPIXEL : Fail to load value.");
    }

    if(!m_Cfg.Get("YPIXEL", m_Param.fitskeys.YPIXEL)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- YPIXEL : Fail to load value.");
    }

    if(!m_Cfg.Get("COMMENT", m_Param.fitskeys.COMMENT)) {
        e = true;
        m_Param.fitskeys.errormsg.push_back("- COMMENT : Fail to load value.");
    }

    if(!e) m_Param.fitskeys.status = true;
}

void freeture::CfgParam::loadMailParam() {

    bool e = false;

    if(!m_Cfg.Get("MAIL_DETECTION_ENABLED", m_Param.mail.MAIL_DETECTION_ENABLED)) {
        e = true;
        m_Param.mail.errormsg.push_back("- MAIL_DETECTION_ENABLED : Fail to load value.");
    }else{

        if(m_Param.mail.MAIL_DETECTION_ENABLED) {

            string mailRecipients;
            if(!m_Cfg.Get("MAIL_RECIPIENT", mailRecipients)) {
                e = true;
                m_Param.mail.errormsg.push_back("- MAIL_RECIPIENT : Fail to load value.");
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
                m_Param.mail.errormsg.push_back("- MAIL_SMTP_SERVER : Fail to load value.");
            }

            string smtp_connection_type;
            if(!m_Cfg.Get("MAIL_CONNECTION_TYPE", smtp_connection_type)) {
                e = true;
                m_Param.mail.errormsg.push_back("- MAIL_CONNECTION_TYPE : Fail to load value.");
            }else {
                try
                {
                    EParser<SmtpSecurity> smtp_security;
                    m_Param.mail.MAIL_CONNECTION_TYPE = smtp_security.parseEnum("MAIL_CONNECTION_TYPE", smtp_connection_type);

                    if(m_Param.mail.MAIL_CONNECTION_TYPE != NO_SECURITY) {

                        if(!m_Cfg.Get("MAIL_SMTP_LOGIN", m_Param.mail.MAIL_SMTP_LOGIN)) {
                            e = true;
                            m_Param.mail.errormsg.push_back("- MAIL_SMTP_LOGIN : Fail to load value.");
                        }

                        if(!m_Cfg.Get("MAIL_SMTP_PASSWORD", m_Param.mail.MAIL_SMTP_PASSWORD)) {
                            e = true;
                            m_Param.mail.errormsg.push_back("- MAIL_SMTP_PASSWORD : Fail to load value.");
                        }
                    }else{
                        m_Param.mail.MAIL_SMTP_LOGIN = "";
                        m_Param.mail.MAIL_SMTP_PASSWORD = "";
                    }

                }catch (std::exception &ex) {
                    e = true;
                    m_Param.mail.errormsg.push_back("- MAIL_CONNECTION_TYPE : " + string(ex.what()));
                }
            }
        }
    }

    if(!e) m_Param.mail.status = true;

}

int freeture::CfgParam::getDeviceID() {
    return m_Param.DEVICE_ID;
}

dataParam freeture::CfgParam::getDataParam() {
    return m_Param.data;
}

logParam freeture::CfgParam::getLogParam() {
    return m_Param.log;
}

framesParam freeture::CfgParam::getFramesParam() {
    return m_Param.framesInput;
}

videoParam freeture::CfgParam::getVidParam() {
    return m_Param.vidInput;
}

cameraParam freeture::CfgParam::getCamParam() {
    return m_Param.camInput;
}

detectionParam freeture::CfgParam::getDetParam() {
    return m_Param.det;
}

stackParam freeture::CfgParam::getStackParam() {
    return m_Param.st;
}

stationParam freeture::CfgParam::getStationParam() {
    return m_Param.station;
}

fitskeysParam freeture::CfgParam::getFitskeysParam() {
    return m_Param.fitskeys;
}

mailParam freeture::CfgParam::getMailParam() {
    return m_Param.mail;
}

parameters freeture::CfgParam::getAllParam() {
    return m_Param;
}

bool freeture::CfgParam::deviceIdIsCorrect() {
    
    if(m_Param.DEVICE_ID == -1) {
        if(showErrors) {
            cout << "Device not valid" << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::dataParamIsCorrect() {
    if(!m_Param.data.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.data.errormsg.size(); i++)
                cout << m_Param.data.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::logParamIsCorrect() {
    if(!m_Param.log.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.log.errormsg.size(); i++)
                cout << m_Param.log.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::framesParamIsCorrect() {

    if(!m_Param.framesInput.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.framesInput.errormsg.size(); i++)
                cout << m_Param.framesInput.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::vidParamIsCorrect() {
    if(!m_Param.vidInput.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.vidInput.errormsg.size(); i++)
                cout << m_Param.vidInput.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::camParamIsCorrect() {
    if(!m_Param.camInput.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.camInput.errormsg.size(); i++)
                cout << m_Param.camInput.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::detParamIsCorrect() {
    if(!m_Param.det.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.det.errormsg.size(); i++)
                cout << m_Param.det.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::stackParamIsCorrect() {
    if(!m_Param.st.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.st.errormsg.size(); i++)
                cout << m_Param.st.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::stationParamIsCorrect() {
    if(!m_Param.station.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.station.errormsg.size(); i++)
                cout << m_Param.station.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::fitskeysParamIsCorrect() {
    if(!m_Param.fitskeys.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.fitskeys.errormsg.size(); i++)
                cout << m_Param.fitskeys.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::mailParamIsCorrect() {
    if(!m_Param.mail.status) {
        if(showErrors) {
            for(int i = 0; i < m_Param.mail.errormsg.size(); i++)
                cout << m_Param.mail.errormsg.at(i) << endl;
        }
        return false;
    }
    return true;
}

bool freeture::CfgParam::inputIsCorrect() {

    switch(m_InputType) {

        case VIDEO :
                return vidParamIsCorrect();
            break;

        case SINGLE_FITS_FRAME :
                return framesParamIsCorrect();
            break;

        // camera
        case CAMERA :
                return camParamIsCorrect();
            break;

    }

    return false;
}

bool freeture::CfgParam::allParamAreCorrect()
{
    //check configuration file
    boost::filesystem::path pcfg(m_CfgFilePath);

    if(!boost::filesystem::exists(pcfg))
    {
       cout << ">> Error. Configuration file not exists at "<< m_CfgFilePath << " use -c to set a different location. " <<endl;
       cout << "Checking command line parameter." << endl;
    }

    bool eFound = false;

    if(!deviceIdIsCorrect())
    {
        eFound = true;
        cout << ">> Errors on device ID. " << endl;
    }

    if(!dataParamIsCorrect()){
        eFound = true;
        cout << ">> Errors on data parameters. " << endl;
    }

    if(!logParamIsCorrect()){
        eFound = true;
        cout << ">> Errors on log parameters. " << endl;
    }

    if(!inputIsCorrect()){
        eFound = true;
        cout << ">> Errors on input parameters. " << endl;
    }

    if(!detParamIsCorrect()){
        eFound = true;
        cout << ">> Errors on detection parameters. " << endl;
    }

    if(!stackParamIsCorrect()){
        eFound = true;
        cout << ">> Errors on stack parameters. " << endl;
    }

    if(!stationParamIsCorrect()){
        eFound = true;
        cout << ">> Errors on station parameters. " << endl;
    }

    if(!fitskeysParamIsCorrect()){
        eFound = true;
        cout << ">> Errors on fitskeys parameters. " << endl;
    }

    if(!mailParamIsCorrect()){
        eFound = true;
        cout << ">> Errors on mail parameters. " << endl;
    }

    if(eFound)
        return false;

    return true;
}
