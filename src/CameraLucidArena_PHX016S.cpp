/**
* \file    CameraLucidArena_PHX016S.cpp
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.0
* \date    03/15/2023
* \brief   Use Arena SDK library to pilot Lucid PHX016S Cameras
*
*/
#include "CameraLucidArena_PHX016S.h"

#include "Logger.h"

#include <cstring>
#include <time.h>
#include <algorithm>
#include <iostream>
#include <boost/date_time.hpp>

#include <opencv2/opencv.hpp>

#include <ArenaApi.h>
#include <SaveApi.h>

#include "ELogSeverityLevel.h"
#include "EParser.h"
#include "Frame.h"
#include "TimeDate.h"
#include "Camera.h"
#include "ArenaSDKManager.h"

using namespace freeture;
using namespace std;

    /**
     * CTor.
     */
    CameraLucidArena_PHX016S::CameraLucidArena_PHX016S(CameraDescription camera_descriptor, cameraParam settings):
        Camera(camera_descriptor, settings)
    {
        m_ExposureAvailable = true;
        m_GainAvailable = true;

        m_MinFPS = MIN_FPS;
        m_MaxFPS = MAX_FPS;
        m_MinGain = MIN_GAIN;
        m_MaxGain = MAX_GAIN;
        m_MinExposure = MIN_US_NORMAL;
        m_MaxExposure = MAX_US_NORMAL;
        m_PixelFormat = PIXEL_FORMAT;
    }

    /**
     * DTor.
     */
    CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S" << endl;
            destroyDevice();
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S; " << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S; " << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S;" << "Unexpected exception thrown" << endl;
        }
    }


    /**
     * FPS
     */
    void CameraLucidArena_PHX016S::getFPSBounds(double &fMin, double &fMax)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getFPSBounds" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            double fpsMin = 0.0;
            double fpsMax = 0.0;

            GenApi::CFloatPtr pAcquisitionFrameRate = m_ArenaDevice->GetNodeMap()->GetNode("AcquisitionFrameRate");

            if (!pAcquisitionFrameRate)
            {
                LOG_ERROR <<"CameraLucidArena_PHX016S::getFPSBounds;" << "AcquisitionFrameRateEnable node not found" << endl;
                return;
            }

            fpsMax = pAcquisitionFrameRate->GetMax();
            fpsMin = pAcquisitionFrameRate->GetMin();

            fMin = fpsMin;
            fMax = fpsMax;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFPSBounds;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFPSBounds;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFPSBounds;" << "Unexpected exception thrown" << endl;
        }
    }

    bool CameraLucidArena_PHX016S::setFPS(double val)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setFPS"<< "(" << val <<")" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            if (val < MIN_FPS)
                val = MIN_FPS;

            if (val > MAX_FPS)
                val = MAX_FPS;

            
            bool pAcquisitionFrameRateEnable = m_ArenaDevice->GetNodeMap()->GetNode("AcquisitionFrameRateEnable");

            if (!pAcquisitionFrameRateEnable)
            {
               LOG_ERROR << "CameraLucidArena_PHX016S::setFPS" << "AcquisitionFrameRateEnable node not found" << endl;
                return false;
            }

            Arena::SetNodeValue<double>(m_ArenaDevice->GetNodeMap(), "AcquisitionFrameRate", val);
            getFPS(m_FPS);

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setFPS;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setFPS;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setFPS;" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    bool CameraLucidArena_PHX016S::getFPS(double& value)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getFPS" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            GenApi::CFloatPtr pAcquisitionFrameRate = m_ArenaDevice->GetNodeMap()->GetNode("AcquisitionFrameRate");

            if (!pAcquisitionFrameRate)
            {
                LOG_ERROR << "CameraLucidArena_PHX016S::getFPS;" << "AcquisitionFrameRate node not found" << endl;
                return false;
            }

            value = pAcquisitionFrameRate->GetValue();

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFPS" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFPS" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFPS" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    /**
     * INFOS
     */
    bool CameraLucidArena_PHX016S::getDeviceInfoBySerial(string serial, Arena::DeviceInfo& device_info)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getDeviceInfoBySerial" << endl;
          
            if (!checkSDK())
                throw runtime_error("SDK not initialized");

            m_ArenaSDKSystem->UpdateDevices(100);

            vector<Arena::DeviceInfo> deviceInfos = m_ArenaSDKSystem->GetDevices();
            size_t n_devices = deviceInfos.size();

            for (int i = 0; i < n_devices; i++) {

                if (string(deviceInfos[i].SerialNumber()) == serial) {
                    device_info = deviceInfos[i];
                    return true;
                }
            }
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getDeviceInfoBySerial" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getDeviceInfoBySerial" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getDeviceInfoBySerial" << "Unexpected exception thrown" << endl;
        }

        LOG_ERROR << "CameraLucidArena_PHX016S::getDeviceInfoBySerial" << "Fail to retrieve camera with this ID." << endl;
        return false;
    }

    string CameraLucidArena_PHX016S::getModelName()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getModelName" << endl;

            if (!checkSDK())
                throw runtime_error("SDK not initialized");

            Arena::DeviceInfo device_info;

            if (!getDeviceInfoBySerial(m_CameraDescriptor.Serial, device_info)) {
                LOG_ERROR << "CameraLucidArena_PHX016S::getModelName" << "Camera not found" << endl;
                return string();
            }

            return device_info.ModelName().c_str();
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getModelName;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getModelName;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getModelName;" << "Unexpected exception thrown" << endl;
        }

        return string();
    }


    /**
     * EXPOSURE
     */
    void CameraLucidArena_PHX016S::getExposureBounds(double& eMin, double& eMax)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getModelName" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            double exposureMin = 0.0;
            double exposureMax = 0.0;

            GenApi::CFloatPtr pExposureTime = m_ArenaDevice->GetNodeMap()->GetNode("ExposureTime");

            if (!pExposureTime)
            {
               LOG_ERROR << "CameraLucidArena_PHX016S::getModelName" << "ExposureTime node not found" << endl;
                return;
            }

            exposureMax = pExposureTime->GetMax();
            exposureMin = pExposureTime->GetMin();

            eMin = exposureMin;
            eMax = exposureMax;

        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getExposureBounds" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getExposureBounds" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getExposureBounds" << "Unexpected exception thrown" << endl;
        }
    }

    double CameraLucidArena_PHX016S::getExposureTime()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getExposureTime" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            GenApi::CFloatPtr pExposureTime = m_ArenaDevice->GetNodeMap()->GetNode("ExposureTime");

            if (!pExposureTime)
            {
                LOG_ERROR << "CameraLucidArena_PHX016S::getExposureTime" << "ExposureTime node not found" << endl;
                return -1;
            }

            double result = pExposureTime->GetValue();

            return result;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getExposureBounds" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getExposureBounds" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getExposureBounds" << "Unexpected exception thrown" << endl;
        }

        return -1.0;
    }

    bool CameraLucidArena_PHX016S::setExposureTime(double val)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setExposureTime (" << val <<")" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");


            double expMin, expMax;

            GenApi::CFloatPtr pExposureTime = m_ArenaDevice->GetNodeMap()->GetNode("ExposureTime");

            if (!pExposureTime)
            {
               LOG_ERROR << "CameraLucidArena_PHX016S::setExposureTime; "<< "ExposureTime node not found" << endl;
                return false;
            }

            expMax = pExposureTime->GetMax();
            expMin = pExposureTime->GetMin();

            if (val >= expMin && val <= expMax)
            {
                m_ExposureTime = val;

                Arena::SetNodeValue<double>(m_ArenaDevice->GetNodeMap(), "ExposureTime", m_ExposureTime);
            }
            else
            {
                LOG_DEBUG << "CameraLucidArena_PHX016S::setExposureTime; " << "> Exposure value (" << val << ") is not in range [ " << expMin << " - " << expMax << " ]" << endl;
                return false;
            }

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setExposureTime" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setExposureTime" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setExposureTime" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    double CameraLucidArena_PHX016S::getMinExposureTime()
    {
        LOG_DEBUG << "CameraLucidArena_PHX016S::getMinExposureTime" << endl;

        return MIN_US_NORMAL;
    }


    /**
     * GAIN
     */
    bool CameraLucidArena_PHX016S::setGain(double val)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setGain" << "(" << val <<")" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");


            double gMin, gMax;

            GenApi::CFloatPtr pGain = m_ArenaDevice->GetNodeMap()->GetNode("Gain");

            if (!pGain)
            {
               LOG_ERROR  << "Gain node not found" << endl;
                return false;
            }


            gMax = pGain->GetMax();
            gMin = pGain->GetMin();


            if ((double)val >= gMin && (double)val <= gMax)
            {
                m_Gain = val;
                Arena::SetNodeValue<double>(m_ArenaDevice->GetNodeMap(), "Gain", m_Gain);
            }
            else
            {
                LOG_ERROR << "> Gain value (" << val << ") is not in range [ " << gMin << " - " << gMax << " ]" << endl;
                return false;
            }

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setGain" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setGain" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setGain" << "Unexpected exception thrown" << endl;
        }


        return false;
    }

    double CameraLucidArena_PHX016S::getGain()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setGain" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");


            GenApi::CFloatPtr pGain = m_ArenaDevice->GetNodeMap()->GetNode("Gain");

            if (!pGain)
            {
                LOG_ERROR << "CameraLucidArena_PHX016S::getGain;" << "Gain node not found" << endl;
                return false;
            }

            return pGain->GetValue();
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getGain" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getGain" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getGain" << "Unexpected exception thrown" << endl;
        }
        return -1;
    }

    void CameraLucidArena_PHX016S::getGainBounds(double& gMin, double& gMax)
    {
         try {
             LOG_DEBUG << "CameraLucidArena_PHX016S::getGainBounds" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            double gainMin = 0.0;
            double gainMax = 0.0;

            GenApi::CFloatPtr pGain = m_ArenaDevice->GetNodeMap()->GetNode("Gain");

            if (!pGain)
            {
                LOG_ERROR << "CameraLucidArena_PHX016S::getGainBounds;" << "Gain node not found" << endl;
                return;
            }

            gainMax = pGain->GetMax();
            gainMin = pGain->GetMin();

            gMin = gainMin;
            gMax = gainMax;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getGainBounds" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getGainBounds" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getGainBounds" << "Unexpected exception thrown" << endl;
        }
    }


    /**
     * FRAME SIZE
     */
    bool CameraLucidArena_PHX016S::getFrameSize(int& x, int& y, int& w, int& h)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getFrameSize" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");


            int64_t ww = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Width");
            int64_t hh = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Height");

            int64_t xx = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "OffsetX");
            int64_t yy = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "OffsetY");

            x = int(xx);
            y = int(yy);
            w = int(ww);
            h = int(hh);

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFrameSize" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFrameSize" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getFrameSize" << "Unexpected exception thrown" << endl;
        }
        return false;
    }

    bool CameraLucidArena_PHX016S::setDefaultFrameSize()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setDefaultFrameSize" << endl;
            
            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            LOG_DEBUG << "CameraLucidArena_PHX016S::setDefaultFrameSize;"<<"Camera sensor size : " << MAX_WIDTH << "x" << MAX_HEIGHT << endl;

            Arena::SetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "OffsetX", 0);
            Arena::SetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "OffsetY", 0);

            Arena::SetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Width", MAX_WIDTH);
            Arena::SetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Height", MAX_HEIGHT);

            m_Width = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Width");
            m_Height = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Height");
            m_StartX = 0;
            m_StartY = 0;

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDefaultFrameSize" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDefaultFrameSize" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDefaultFrameSize" << "Unexpected exception thrown" << endl;
        }

        return false;
    }
    
    bool CameraLucidArena_PHX016S::setFrameSize()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setFrameSize" << endl;

            if ( !checkSDKDevice() )
                throw runtime_error("SDK not initialized");

            if (m_StartX % 2 != 0)
                throw runtime_error("X Offset must be multiple of 2");

            Arena::SetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Width", m_Width);
            Arena::SetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Height", m_Height);

            Arena::SetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "OffsetX", m_StartX);
            Arena::SetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "OffsetY", m_StartY);

            m_StartX = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "OffsetX");
            m_StartY = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "OffsetY");

            m_Width = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Width");
            m_Height = Arena::GetNodeValue<int64_t>(m_ArenaDevice->GetNodeMap(), "Height");

            LOG_DEBUG << "CameraLucidArena_PHX016S::setFrameSize;" << "Camera region size : " << m_Width << "x" << m_Height << "("<< m_StartX<<","<< m_StartY<<")" << endl;

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setFrameSize;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setFrameSize;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setFrameSize;" << "Unexpected exception thrown" << endl;
        }
        return false;
    }


    /**
     * PIXEL FORMAT
     */
    bool CameraLucidArena_PHX016S::getPixelFormat(CamPixFmt& format)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getPixelFormat" << endl;
          
            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");


            GenICam::gcstring _pixFormat = Arena::GetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "PixelFormat");

            string pixFormat(_pixFormat.c_str());

            if (pixFormat == "Mono8")
                format = CamPixFmt::MONO8;
            else
                if (pixFormat == "Mono12")
                    format = CamPixFmt::MONO12;
                else
                    if (pixFormat == "Mono16")
                        format = CamPixFmt::MONO16;
                    else
                        return false;


            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::getPixelFormat" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getPixelFormat" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::getPixelFormat" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    bool CameraLucidArena_PHX016S::setPixelFormat()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setPixelFormat" << endl;
           
            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            switch (m_PixelFormat)
            {

            case CamPixFmt::MONO8:
            {
                Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "PixelFormat", "Mono8");
            }
            break;

            case CamPixFmt::MONO12:
            {
                Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "PixelFormat", "Mono12");
            }
            break;
            case  CamPixFmt::MONO16:
            {
                Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "PixelFormat", "Mono16");
            }
            break;
            }



            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setPixelFormat" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setPixelFormat" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setPixelFormat" << "Unexpected exception thrown" << endl;
        }
        return false;
    }

    void CameraLucidArena_PHX016S::getAvailablePixelFormats()
    {
        LOG_DEBUG << "CameraLucidArena_PHX016S::getAvailablePixelFormats" << endl;

        LOG_INFO << ">> Device pixel formats (firmware ver: 1.65.0 - 01/2023  :" << endl;
        LOG_INFO << "- Mono8" << endl;
        LOG_INFO << "- Mono10" << endl;
        LOG_INFO << "- Mono10p" << endl;
        LOG_INFO << "- Mono10Packed" << endl;
        LOG_INFO << "- Mono12" << endl;
        LOG_INFO << "- Mono12p" << endl;
        LOG_INFO << "- Mono12Packed" << endl;
        LOG_INFO << "- Mono16" << endl;
        LOG_INFO << "- Mono24" << endl;
        LOG_INFO << "- PolarizeMono8" << endl;
        LOG_INFO << "- PolarizeMono12" << endl;
        LOG_INFO << "- PolarizeMono12p" << endl;
        LOG_INFO << "- PolarizeMono12Packed" << endl;
        LOG_INFO << "- PolarizeMono16" << endl;
        LOG_INFO << "- BayerGR8" << endl;
        LOG_INFO << "- BayerRG8" << endl;
        LOG_INFO << "- BayerGB8" << endl;
        LOG_INFO << "- BayerBG8" << endl;
        LOG_INFO << "- BayerGR10" << endl;
        LOG_INFO << "- BayerRG10" << endl;
        LOG_INFO << "- BayerGB10" << endl;
        LOG_INFO << "- BayerBG10" << endl;
        LOG_INFO << "- BayerGR10p" << endl;
        LOG_INFO << "- BayerRG10p" << endl;
        LOG_INFO << "- BayerGB10p" << endl;
        LOG_INFO << "- BayerBG10p" << endl;
        LOG_INFO << "- BayerGR10Packed" << endl;
        LOG_INFO << "- BayerRG10Packed" << endl;
        LOG_INFO << "- BayerGB10Packed" << endl;
        LOG_INFO << "- BayerBG10Packed" << endl;
        LOG_INFO << "- BayerGR12" << endl;
        LOG_INFO << "- BayerRG12" << endl;
        LOG_INFO << "- BayerGB12" << endl;
        LOG_INFO << "- BayerBG12" << endl;
        LOG_INFO << "- BayerGR12p" << endl;
        LOG_INFO << "- BayerRG12p" << endl;
        LOG_INFO << "- BayerGB12p" << endl;
        LOG_INFO << "- BayerBG12p" << endl;
        LOG_INFO << "- BayerGR12Packed" << endl;
        LOG_INFO << "- BayerRG12Packed" << endl;
        LOG_INFO << "- BayerGB12Packed" << endl;
        LOG_INFO << "- BayerBG12Packed" << endl;
        LOG_INFO << "- BayerGR16" << endl;
        LOG_INFO << "- BayerRG16" << endl;
        LOG_INFO << "- BayerGB16" << endl;
        LOG_INFO << "- BayerBG16" << endl;
        LOG_INFO << "- BayerGR24" << endl;
        LOG_INFO << "- BayerRG24" << endl;
        LOG_INFO << "- BayerGB24" << endl;
        LOG_INFO << "- BayerBG24" << endl;
        LOG_INFO << "- RGB8" << endl;
        LOG_INFO << "- RGB12p" << endl;
        LOG_INFO << "- RGB16" << endl;
        LOG_INFO << "- RGB24" << endl;
        LOG_INFO << "- BGR8" << endl;
        LOG_INFO << "- BGR12p" << endl;
        LOG_INFO << "- BGR16" << endl;
        LOG_INFO << "- BGR24" << endl;
        LOG_INFO << "- YCbCr8" << endl;
        LOG_INFO << "- YCbCr8_CbYCr" << endl;
        LOG_INFO << "- YUV422_8" << endl;
        LOG_INFO << "- YUV422_8_UYVY" << endl;
        LOG_INFO << "- YCbCr411_8" << endl;
        LOG_INFO << "- YUV411_8_UYYVYY" << endl;
        LOG_INFO << "- PolarizedAngles_0d_45d_90d_135d_Mono8" << endl;
        LOG_INFO << "- PolarizedAngles_0d_45d_90d_135d_Mono12p" << endl;
        LOG_INFO << "- PolarizedAngles_0d_45d_90d_135d_Mono16" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_Mono8" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_Mono12p" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_Mono16" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_S3_Mono8" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_S3_Mono12p" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_S3_Mono16" << endl;
        LOG_INFO << "- PolarizedDolpAolp_Mono8" << endl;
        LOG_INFO << "- PolarizedDolpAolp_Mono12p" << endl;
        LOG_INFO << "- PolarizedDolp_Mono8" << endl;
        LOG_INFO << "- PolarizedDolp_Mono12p" << endl;
        LOG_INFO << "- PolarizedAolp_Mono8" << endl;
        LOG_INFO << "- PolarizedAolp_Mono12p" << endl;
        LOG_INFO << "- PolarizedAngles_0d_45d_90d_135d_BayerRG8" << endl;
        LOG_INFO << "- PolarizedAngles_0d_45d_90d_135d_BayerRG12p" << endl;
        LOG_INFO << "- PolarizedAngles_0d_45d_90d_135d_BayerRG16" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_BayerRG8" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_BayerRG12p" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_BayerRG16" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_S3_BayerRG8" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_S3_BayerRG12p" << endl;
        LOG_INFO << "- PolarizedStokes_S0_S1_S2_S3_BayerRG16" << endl;
        LOG_INFO << "- PolarizedDolpAolp_BayerRG8" << endl;
        LOG_INFO << "- PolarizedDolpAolp_BayerRG12p" << endl;
        LOG_INFO << "- PolarizedDolp_BayerRG8" << endl;
        LOG_INFO << "- PolarizedDolp_BayerRG12p" << endl;
        LOG_INFO << "- PolarizedAolp_BayerRG8" << endl;
        LOG_INFO << "- PolarizedAolp_BayerRG12p" << endl;
        LOG_INFO << "- Raw24" << endl;
        LOG_INFO << "- Raw16" << endl;
        LOG_INFO << "- YCbCr422_8_CbYCrY" << endl;
        LOG_INFO << "- YCbCr422_16_CbYCrY" << endl;
        LOG_INFO << "- YCbCr422_24_CbYCrY" << endl;
        LOG_INFO << "- YCbCr411_8_CbYYCrYY" << endl;
        LOG_INFO << "- YCbCr411_16_CbYYCrYY" << endl;
        LOG_INFO << "- YCbCr411_24_CbYYCrYY" << endl;
        LOG_INFO << "- PolarizedDolpAngle_Mono8" << endl;
        LOG_INFO << "- PolarizedDolpAngle_Mono12p" << endl;
        LOG_INFO << "- PolarizedDolpAngle_Mono16" << endl;
        LOG_INFO << "- PolarizedDolpAngle_BayerRG8" << endl;
        LOG_INFO << "- PolarizedDolpAngle_BayerRG12p" << endl;
        LOG_INFO << "- PolarizedDolpAngle_BayerRG16" << endl;

        // Compare found pixel formats to currently formats supported by freeture

        LOG_INFO << endl << ">> Available pixel formats :" << endl;
        LOG_INFO << "- MONO8 available --> ID : Mono8 " << endl;
        LOG_INFO << "- MONO12 available --> ID : Mono12 " << endl;
        LOG_INFO << "- MONO16 available --> ID : Mono16" << endl;

    }

    /**
     * MODE
     */
    void CameraLucidArena_PHX016S::acqStop()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::acqStop" << endl;

            if (m_ArenaDevice == nullptr)
            {
                LOG_ERROR << "CameraLucidArena_PHX016S::acqStop;" << "Camera is nullptr" << endl;
                return;
            }

            //arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

            //cout << "Completed buffers = " << (unsigned long long) nbCompletedBuffers  ;
            //cout << "Failures          = " << (unsigned long long) nbFailures          ;
            //cout << "Underruns         = " << (unsigned long long) nbUnderruns         ;

            //LOG_INFO << "Completed buffers = " << (unsigned long long) nbCompletedBuffers;
            //LOG_INFO << "Failures          = " << (unsigned long long) nbFailures;
            //LOG_INFO << "Underruns         = " << (unsigned long long) nbUnderruns;

            LOG_INFO << "Stopping acquisition..." << endl;

            m_ArenaDevice->StopStream();

            m_Streaming = false;

            LOG_INFO << "Acquisition stopped." << endl;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::acqStop" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::acqStop" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::acqStop" << "Unexpected exception thrown" << endl;
        }
    }

    void CameraLucidArena_PHX016S::grabCleanse()
    {
        LOG_DEBUG << "CameraLucidArena_PHX016S::grabCleanse; *** DO NOTHING ***" << endl;
    }

    bool CameraLucidArena_PHX016S::grabImage(shared_ptr<Frame> newFrame)
    {
        try
        {
            if (LOG_SPAM_FRAME_STATUS)
                LOG_DEBUG << "CameraLucidArena_PHX016S::grabImage" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");


            Arena::IImage* pOriginalImage = m_ArenaDevice->GetImage(IMAGE_TIMEOUT);

            if (!pOriginalImage->HasImageData()) {
                LOG_ERROR << "Image has not image data" << endl;
            }

            if (pOriginalImage->HasChunkData()) {
                LOG_ERROR << "Image has data chunk" << endl;
            }

            newFrame->mDate = TimeDate::Date(boost::posix_time::microsec_clock::universal_time());

            Arena::IImage* pCopiedImage = Arena::ImageFactory::Create(pOriginalImage->GetData(),
                pOriginalImage->GetPayloadSize(), pOriginalImage->GetWidth(),
                pOriginalImage->GetHeight(),
                pOriginalImage->GetPixelFormat());

            m_ArenaDevice->RequeueBuffer(pOriginalImage);

            size_t payload_size = pCopiedImage->GetPayloadSize();
            size_t size = pCopiedImage->GetSizeFilled();
            size_t width = pCopiedImage->GetWidth();
            size_t height = pCopiedImage->GetHeight();
            size_t size_of_buffer = pCopiedImage->GetSizeOfBuffer();

            if (pCopiedImage->IsIncomplete()) {
                LOG_ERROR << "Image is incomplete: " << " size_filled=" << size << "<= payload size= " << payload_size << "<= size_of_buffer" << size_of_buffer << endl;
                LOG_ERROR << "this is likely due to missed packets or a small buffer" << endl;
                getStreamMissedPacketCount();
            }

            string m_PixelFormat = GetPixelFormatName(static_cast<PfncFormat>(pCopiedImage->GetPixelFormat()));

            uint64_t timestampNs = pCopiedImage->GetTimestampNs();

            if (LOG_SPAM_FRAME_STATUS)
                LOG_DEBUG << " (" << " Gain " << m_Gain << "; FPS " << m_FPS << "; Exposure " << m_ExposureTime << "; " << size << " bytes; " << width << "x" << height << "; " << m_PixelFormat << "; timestamp (ns): " << timestampNs << ")" << endl;

            const uint8_t* u_buffer_data = pCopiedImage->GetData();

            size_t buffer_size = pCopiedImage->GetSizeFilled();

            CopyFrame(newFrame, u_buffer_data, buffer_size);
            Arena::ImageFactory::Destroy(pCopiedImage);

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::grabImage;" << e.what() << endl;
            disconnect();
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::grabImage;" << "Standard exception thrown: " << ex.what() << endl;
            disconnect();
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::grabImage;" << "Unexpected exception thrown" << endl;
            disconnect();
        }
        return false;
    }

    bool CameraLucidArena_PHX016S::acqStart()
    {
        LOG_DEBUG << "CameraLucidArena_PHX016S::acqStart()" << endl;

        return acqStart(true);
    }

    bool CameraLucidArena_PHX016S::acqStart(bool continuous)
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::acqStart;" << "continuous=" << continuous << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            if (continuous) {
                // Disable stream packet resend
                //    Enable stream packet resend before starting the stream. Images are
                //    sent from the camera to the host in packets using UDP protocol,
                //    which includes a header image number, packet number, and timestamp
                //    information. If a packet is missed while receiving an image, a
                //    packet resend is requested and this information is used to retrieve
                //    and redeliver the missing packet in the correct order.

                LOG_INFO << "CameraLucidArena_PHX016S::grabInitialization;" << "Disable stream packet resend" << endl;
                Arena::SetNodeValue<bool>(m_ArenaDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", false);

                // Set acquisition mode
                //    Set acquisition mode before starting the stream. Starting the stream
                //    requires the acquisition mode to be set beforehand. The acquisition
                //    mode controls the number of images a device acquires once the stream
                //    has been started. Setting the acquisition mode to 'Continuous' keeps
                //    the stream from stopping. This example returns the camera to its
                //    initial acquisition mode near the end of the example.
                LOG_INFO << "CameraLucidArena_PHX016S::acqStart;" << "Set camera to CONTINUOUS MODE" << endl;
                Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "AcquisitionMode", "Continuous");
            }
            else
            {
                // Disable stream packet resend
                //    Enable stream packet resend before starting the stream. Images are
                //    sent from the camera to the host in packets using UDP protocol,
                //    which includes a header image number, packet number, and timestamp
                //    information. If a packet is missed while receiving an image, a
                //    packet resend is requested and this information is used to retrieve
                //    and redeliver the missing packet in the correct order.

                LOG_INFO << "CameraLucidArena_PHX016S::grabInitialization;" << "Enable stream packet resend" << endl;
                Arena::SetNodeValue<bool>(m_ArenaDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

                LOG_INFO << "CameraLucidArena_PHX016S::acqStart;" << "Set camera to SINGLEFRAME" << endl;
                Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "AcquisitionMode", "SingleFrame");
            }

            // Start stream
            //    Start the stream before grabbing any images. Starting the stream
            //    allocates buffers, which can be passed in as an argument (default: 10),
            //    and begins filling them with data. Starting the stream blocks write
            //    access to many features such as width, height, and pixel format, as
            //    well as acquisition and buffer handling modes, among others. The stream
            //    needs to be stopped later.
            LOG_INFO << "CameraLucidArena_PHX016S::acqStart;" << "Start acquisition on camera" << endl;
            m_ArenaDevice->StartStream();
           
            m_Streaming = true;

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::acqStart;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::acqStart;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::acqStart;" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    /**
     * INTERNALS
     */

    /// <summary>
    /// Set continuous mode ans set FPS to current rate
    /// </summary>
    bool CameraLucidArena_PHX016S::setContinuousMode() {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setContinuousMode" << endl;
            Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "AcquisitionMode", "Continuous");
            setFPS(m_CameraSettings.ACQ_FPS);
            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setContinuousMode" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setContinuousMode" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setContinuousMode" << "Unexpected exception thrown" << endl;
        }
        return false;
    }

    /// <summary>
    /// Set single shot mode and MIN_FPS (0.1)
    /// </summary>
    bool CameraLucidArena_PHX016S::setSingleShotMode()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setSingleShotMode" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "AcquisitionMode", "SingleFrame");

            if (!setFPS(MIN_FPS))
                return false;

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setSingleShotMode" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setSingleShotMode" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setSingleShotMode" << "Unexpected exception thrown" << endl;
        }
        return false;
    }

    bool  CameraLucidArena_PHX016S::setDayContinuous()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setDayContinuous" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            setContinuousMode();
            setGain(m_CameraSettings.ACQ_DAY_GAIN);
            setExposureTime(m_CameraSettings.ACQ_DAY_EXPOSURE);
            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDayContinuous" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDayContinuous" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDayContinuous" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    bool  CameraLucidArena_PHX016S::setNightContinuous() {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setNightContinuous" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            setContinuousMode();
            setGain(m_CameraSettings.ACQ_NIGHT_GAIN);
            setExposureTime(m_CameraSettings.ACQ_NIGHT_EXPOSURE);
            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setNightContinuous" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setNightContinuous" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setNightContinuous" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    bool CameraLucidArena_PHX016S::setDayRegular() {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setDayRegular" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            setSingleShotMode();
            setGain(m_CameraSettings.ACQ_DAY_GAIN);
            setExposureTime(m_CameraSettings.ACQ_DAY_EXPOSURE);
            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDayRegular" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDayRegular" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setDayRegular" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    bool  CameraLucidArena_PHX016S::setNightRegular() {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::setNightRegular" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            setSingleShotMode();
            setGain(m_CameraSettings.regcap.ACQ_REGULAR_CFG.gain);
            setExposureTime(m_CameraSettings.regcap.ACQ_REGULAR_CFG.exp);
            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::setNightRegular" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setNightRegular" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::setNightRegular" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    void CameraLucidArena_PHX016S::getStreamMissedPacketCount()
    {
        try
        {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getStreamMissedPacketCount" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            GenApi::INode* pNode = m_ArenaDevice->GetTLStreamNodeMap()->GetNode("StreamMissedPacketCount");
            if (pNode)
            {
                GenApi::CIntegerPtr val = pNode;
                m_MissingPacketCount = val->GetValue();
                if (m_PreviousMissingPacketCount != m_MissingPacketCount) {
                    LOG_WARNING << "Missing packet count was " << m_PreviousMissingPacketCount << " now is " << m_MissingPacketCount;
                    m_PreviousMissingPacketCount = m_MissingPacketCount;
                }
            }
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S;" << "Unexpected exception thrown" << endl;
        }
    }

    /// <summary>
    ///  DeviceTemperatureSelector
    ///  *DeviceTemperature
    ///    EnumEntry : 'Sensor1' (Not available)
    ///    EnumEntry : 'Sensor0' (Not available)
    ///    EnumEntry : 'TEC' (Not available)
    ///    EnumEntry : 'Sensor'
    /// </summary>
    /// <returns></returns>
    bool CameraLucidArena_PHX016S::getTemperature(string selector)
    {
        try
        {
            LOG_DEBUG << "CameraLucidArena_PHX016S::getTemperature" << endl;


            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            GenICam::gcstring value = GenICam::gcstring(selector.c_str());

            Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "DeviceTemperatureSelector", value);

            //sensor
            GenApi::CEnumerationPtr  pEnumeration = m_ArenaDevice->GetNodeMap()->GetNode("DeviceTemperatureSelector");
            GenApi::CEnumEntryPtr pCurrentEntry = pEnumeration->GetCurrentEntry();
            GenICam::gcstring currentEntrySymbolic = pCurrentEntry->GetSymbolic();

            GenApi::CFloatPtr t = m_ArenaDevice->GetNodeMap()->GetNode("DeviceTemperature");

            double temperature = t->GetValue();
            LOG_INFO << "CameraLucidArena_PHX016S::getTemperature;" << currentEntrySymbolic << "=" << temperature << "[C]" << endl;
            m_LastTemperature = temperature;
            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << "Unexpected exception thrown";
        }

        return false;
    }

    double CameraLucidArena_PHX016S::getTemperature()
    {
        if (!getTemperature("Sensor"))
            return std::numeric_limits<double>::quiet_NaN();
        return m_LastTemperature;
    }

    void CameraLucidArena_PHX016S::CopyFrame(shared_ptr<Frame> frame, const uint8_t* u_buffer_data, size_t buffer_size)
    {
        try
        {
            if (LOG_SPAM_FRAME_STATUS)
                LOG_DEBUG << "CameraLucidArena_PHX016S::CopyFrame" << endl;

            frame->SetImage(u_buffer_data, buffer_size);

            shared_ptr<cv::Mat> image = make_shared<cv::Mat>();
            int saturateVal = 0;

            switch (m_PixelFormat)
            {
            case CamPixFmt::MONO8:
                {
                    //BOOST_LOG_SEV(logger, normal) << "Creating Mat 8 bits ...";
                    image = make_shared<cv::Mat>(m_Height, m_Width, CV_8UC1, frame->getData());
                    saturateVal = 255;
                    break;
                }
            case CamPixFmt::MONO12:
                {

                    //BOOST_LOG_SEV(logger, normal) << "Creating Mat 16 bits ...";
                    image = make_shared<cv::Mat>(m_Height, m_Width, CV_16UC1, frame->getData());
                    saturateVal = 4095;

                    if (m_ShiftBitsImage) {

                        unsigned short* p;

                        for (int i = 0; i < image->rows; i++) {
                            p = image->ptr<unsigned short>(i);
                            for (int j = 0; j < image->cols; j++)
                                p[j] = p[j] >> 4;
                        }
                    }
                    break;
                }
                case CamPixFmt::MONO16:
                {
                    image = make_shared<cv::Mat>(m_Height, m_Width, CV_16UC1, frame->getData());
                    saturateVal = 65535;
                }
            }

            frame->Image = image;
            frame->mFps = m_FPS;
            frame->mFormat = m_PixelFormat;
            frame->mDate = TimeDate::splitIsoExtendedDate(to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()));
            frame->mSaturatedValue = saturateVal;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::CopyFrame;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::CopyFrame;" << "Unexpected exception thrown" << endl;
        }
    }

    bool CameraLucidArena_PHX016S::checkSDKDevice()
    {
        if (!checkSDK())
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::checkSDKDevice;" << "SDK not initialized" << endl;
            return false;
        }

        if (m_ArenaDevice == nullptr)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::checkSDKDevice;" << "Device not created" << endl;
            return false;
        }

//         if (! m_ArenaDevice->IsConnected() ) {
//             LOG_ERROR << "CameraLucidArena_PHX016S::checkSDKDevice;" << "Camera is not connected";
//             return false;
//         }

        return true;
    }

    bool  CameraLucidArena_PHX016S::checkSDK()
    {

        if (m_ArenaSDKSystem == nullptr)
        {
            return false;
        }

        return true;
    }







    /*
     *   FAB METHODS
     */
   /**
    * Create a Lucid camera using serial number
    */
    bool CameraLucidArena_PHX016S::createDevice()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::createDevice" << endl;

            if (!checkSDK())
                throw runtime_error("SDK not initialized");

            Arena::DeviceInfo target_device_info;

            if (!getDeviceInfoBySerial(m_CameraDescriptor.Serial, target_device_info)) {
                LOG_ERROR << "Camera not found" << endl;
                return false;
            }

            LOG_DEBUG << "CameraLucidArena_PHX016S::createDevice; " << "DEVICE NAME " << target_device_info.ModelName() << endl;

            m_ArenaDevice = m_ArenaSDKSystem->CreateDevice(target_device_info);

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            m_Connected = true;

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::createDevice;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::createDevice;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::createDevice;" << "Unexpected exception thrown" << endl;
        }
        return false;
    }

    bool CameraLucidArena_PHX016S::initSDK()
    {
        try
        {
            LOG_DEBUG << "CameraLucidArena_PHX016S::initSDK;" << "Retrieve Arena SDK instance" << endl;
            m_ArenaSDKSystem = ArenaSDKManager::Get();

            if (!checkSDK())
                return false;

            return true;

        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::initSDK" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::initSDK" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::initSDK" << "Unexpected exception thrown" << endl;
        }
        return false;
    }

    bool CameraLucidArena_PHX016S::initOnce()
    {
        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce" << endl;

#if ARENA_TOOLS
        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "EXPLORING CAMERA MAP - THIS WILL NOT BE PART OF A PRODUCTION BUILD" << endl;
        ArenaSDKManager::exploreNodeMaps(m_ArenaDevice);
#endif

        // select user set 1
        ArenaSDKManager::setStringValue(m_ArenaDevice, "UserSetSelector", "UserSet1");

        //this is an enumeration Off - Continuous
        //LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "ExposureAutoLowerLimit = Off";
        //ArenaSDKManager::setStringValue(m_ArenaDevice, "ExposureAutoLowerLimit", "Off");

        //this is an enumeration Off - Continuous  - Once
        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "ExposureAuto = Off" << endl;
        ArenaSDKManager::setStringValue(m_ArenaDevice, "ExposureAuto", "Off");

        //this is an enumeration Off - Continuous
        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "ExposureAutoLimitAuto = Off" << endl;
        ArenaSDKManager::setStringValue(m_ArenaDevice, "ExposureAutoLimitAuto", "Off");

        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "AcquisitionFrameRateEnable = true" << endl;
        ArenaSDKManager::setBooleanValue(m_ArenaDevice, "AcquisitionFrameRateEnable", true);

        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "GammaEnable = false";
        ArenaSDKManager::setBooleanValue(m_ArenaDevice, "GammaEnable", false);

        LOG_INFO << "CameraLucidArena_PHX016S::grabInitialization;" << "Enable stream to auto negotiate packet size" << endl;
        Arena::SetNodeValue<bool>(m_ArenaDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", false);

        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "DeviceStreamChannelPacketSize = 1500" << endl;
        ArenaSDKManager::setIntegerValue(m_ArenaDevice, "DeviceStreamChannelPacketSize", 1500);

        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "AcquisitionFrameRate = 0.1" << endl;
        ArenaSDKManager::setFloatValue(m_ArenaDevice, "AcquisitionFrameRate", 0.1);
        
        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "UserSetDefault = UserSet1" << endl;
        ArenaSDKManager::setStringValue(m_ArenaDevice, "UserSetDefault", "UserSet1");

        // execute the save
        LOG_DEBUG << "CameraLucidArena_PHX016S::initOnce;" << "Send command: UserSetSave" << endl;
        ArenaSDKManager::sendCommand(m_ArenaDevice, "UserSetSave");
      
        return true;
    }

    bool CameraLucidArena_PHX016S::init()
    {
        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::init;" << "CameraLucidArena_PHX016S::init" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            //get temperature
            if (!getTemperature("Sensor"))
                throw runtime_error("Error getting camera sensor temperature");
            LOG_INFO << "CameraLucidArena_PHX016S::init;" << "TEMPERATURE\t:" << m_LastTemperature << "[C]" << endl;


            //set pixel format
            EParser<CamPixFmt> px_fmt_parser;
            string fstring = px_fmt_parser.getStringEnum(static_cast<CamPixFmt>(m_PixelFormat));
            if (!setPixelFormat())
                throw runtime_error("Error setting pixel format");
            LOG_INFO << "CameraLucidArena_PHX016S::init;" << "PIXEL FORMAT\t\t:" << fstring << endl;


            //set frame size
            if (!setFrameSize())
                throw runtime_error("Error setting frame size");
            LOG_INFO << "CameraLucidArena_PHX016S::init;" << "FRAME SIZE \t\t:" << m_Width<<"x"<<m_Height<<" ("<<m_StartX<<","<<m_StartY<<")" << endl;


            //set fps
            if (!setFPS(MIN_FPS))
                throw runtime_error("Error setting minimum FPS value");
            LOG_INFO << "CameraLucidArena_PHX016S::init;" << "FPS\t\t:" << m_FPS << endl;


            //set exposure time
            if (!setExposureTime(MIN_US_NORMAL))
                throw runtime_error("Error setting minimum exposure time value");
            LOG_INFO << "CameraLucidArena_PHX016S::init;" << "EXPOSURE\t:" << m_ExposureTime << endl;


            //set gain
            if (!setGain(MIN_GAIN))
                throw runtime_error("Error setting minimum gain value");
            LOG_INFO << "CameraLucidArena_PHX016S::init;" << "GAIN\t\t:" << m_Gain << endl;


            try {
            
              
                // Set buffer handling mode
                //    Set buffer handling mode before starting the stream. Starting the
                //    stream requires the buffer handling mode to be set beforehand. The
                //    buffer handling mode determines the order and behavior of buffers in
                //    the underlying stream engine. Setting the buffer handling mode to
                //    'NewestOnly' ensures the most recent image is delivered, even if it
                //    means skipping frames.

                LOG_DEBUG << "CameraLucidArena_PHX016S::grabInitialization;" << "Set buffer handling mode to 'NewestOnly'" << endl;
                Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");

                // Enable stream auto negotiate packet size
                //    Setting the stream packet size is done before starting the stream.
                //    Setting the stream to automatically negotiate packet size instructs
                //    the camera to receive the largest packet size that the system will
                //    allow. This generally increases frame rate and results in fewer
                //    interrupts per image, thereby reducing CPU load on the host system.
                //    Ethernet settings may also be manually changed to allow for a
                //    larger packet size.

                LOG_DEBUG << "CameraLucidArena_PHX016S::grabInitialization;" << "Set camera TriggerMode to Off" << endl;
                Arena::SetNodeValue<GenICam::gcstring>(m_ArenaDevice->GetNodeMap(), "TriggerMode", "Off");

                return true;
            }
            catch (GenICam::GenericException& e) {
                LOG_ERROR << "CameraLucidArena_PHX016S::grabInitialization;" << e.what() << endl;
            }
            catch (std::exception& ex)
            {
                LOG_ERROR << "CameraLucidArena_PHX016S::grabInitialization;" << "Standard exception thrown: " << ex.what() << endl;
            }
            catch (...)
            {
                LOG_ERROR << "CameraLucidArena_PHX016S::grabInitialization;" << "Unexpected exception thrown" << endl;
            }
            

        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::init; " << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::init; " << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::init; " << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    bool CameraLucidArena_PHX016S::grabInitialization()
    {
        return true;
    }

    bool CameraLucidArena_PHX016S::configurationCheck(parameters&)
    {

        try {
            LOG_DEBUG << "CameraLucidArena_PHX016S::configurationCheck" << endl;

            if (!checkSDKDevice())
                throw runtime_error("SDK not initialized");

            //SET CONTINUOUS MODE 
            if (!setDayContinuous()) {
                LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << "Set DAY CONTINUOUS configuration failed" << endl;
                return false;
            }


            if (!setNightContinuous()) {
                LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << "Set NIGHT REGULAR configuration failed" << endl;
                return false;

            }

            //SET SINGLESHOT MODE
            if (!setDayRegular()) {
                LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << "Set DAY REGULAR configuration failed" << endl;
                return false;
            }

            if (!setNightRegular()) {
                LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << "Set NIGHT REGULAR configuration failed";
                return false;

            }

            return true;
        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::configurationCheck;" << "Unexpected exception thrown" << endl;
        }

        return false;
    }

    void CameraLucidArena_PHX016S::configure(parameters&)
    {
        LOG_DEBUG << "CameraLucidArena_PHX016S::configure" << endl;
    }

    void CameraLucidArena_PHX016S::fetchBounds(parameters&)
    {
        try{
        LOG_DEBUG << "CameraLucidArena_PHX016S::fetchBounds" << endl;

        if (!checkSDKDevice())
            throw runtime_error("SDK not initialized");

        //fetch gain bounds
        getGainBounds(m_MinGain,m_MaxGain);

        //fetch fps bounds
        getFPSBounds(m_MinFPS,m_MaxFPS);

        //fetch exposure bounds
        getExposureBounds(m_MinExposure,m_MaxExposure);

        }
        catch (GenICam::GenericException& e) {
            LOG_ERROR << "CameraLucidArena_PHX016S::fetchBounds;" << e.what() << endl;
        }
        catch (std::exception& ex)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::fetchBounds;" << "Standard exception thrown: " << ex.what() << endl;
        }
        catch (...)
        {
            LOG_ERROR << "CameraLucidArena_PHX016S::fetchBounds;" << "Unexpected exception thrown" << endl;
        }

    }

    bool CameraLucidArena_PHX016S::destroyDevice()
    {
        LOG_DEBUG << "CameraLucidArena_PHX016S::destroyDevice" << endl;

        if (!checkSDK())
            throw runtime_error("SDK not initialized");

        try
        {
            if (m_ArenaDevice != nullptr)
            {

                if (m_Streaming)
                    acqStop();

                LOG_DEBUG << "CameraLucidArena_PHX016S::destroyDevice;" << "Deallocating Arena Device" << endl;
                m_ArenaSDKSystem->DestroyDevice(m_ArenaDevice);

                m_Connected = false;
                return true;
            }
        }
        catch (...)
        {
            m_Connected = false;
        }

        return false;
    }

    bool CameraLucidArena_PHX016S::reset()
    {
        // execute the save
        LOG_DEBUG << "CameraLucidArena_PHX016S::reset;" << "Perform device reset" << endl;
        if (!ArenaSDKManager::sendCommand(m_ArenaDevice, "DeviceReset")) {
            LOG_ERROR << "CameraLucidArena_PHX016S::reset;" << "Reset failed" << endl;
            return false;
        }
        else 
            LOG_DEBUG << "CameraLucidArena_PHX016S::reset;" << "OK" << endl;

        return true;
    }

    std::string CameraLucidArena_PHX016S::getModel()
    {
        return MODEL "(" CHIPSET_MODEL ")";
    }

    bool CameraLucidArena_PHX016S::isConnected()
    {
        return m_Connected;
    }

    bool CameraLucidArena_PHX016S::connect()
    {
        bool destroyed, created;

        destroyed = destroyDevice();
        created = createDevice();

        return m_Connected;
    }

    bool CameraLucidArena_PHX016S::disconnect()
    {
        return destroyDevice();
    }
