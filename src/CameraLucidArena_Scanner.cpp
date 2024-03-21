/**
* \file    CameraGigeAravis_Scanner.cpp
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.0
* \date    02/21/2024
* \brief   Use Arena SDK library to scan Lucid Arena Phoenix Cameras
*
*/
#include "CameraLucidArena_Scanner.h"

#include "Logger.h"

#include <iostream>


#include "ArenaSDKManager.h"
#include <ArenaApi.h>
#include <SaveApi.h>
#include <GenICam.h>


using namespace std;
using namespace GenICam;
using namespace freeture;

CameraLucidArena_Scanner::CameraLucidArena_Scanner(CamSdkType sdk) : CameraScanner(sdk)
{
    m_ArenaSDKSystem = ArenaSDKManager::Get();
}

CameraLucidArena_Scanner::~CameraLucidArena_Scanner()
{
    
}

/**
 * Called to get the current reachable cameras.
 * Lucid cameras are GigEVision cameras and a specific tag into the name. "PHX016S"
 */
void CameraLucidArena_Scanner::UpdateCameraList()
{
    LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList";
    Devices.clear();
    try
    {

        if (m_ArenaSDKSystem == nullptr) 
        {
            LOG_ERROR << "CameraLucidArena_Scanner::UpdateCameraList; Arena::OpenSystem returned nullptr" << endl;
            throw std::runtime_error("Arena::OpenSystem returned nullptr");
        }

        LOG_INFO  << "CameraLucidArena_Scanner::UpdateCameraList; Arena SDK Version: " << Arena::GetVersion().c_str() << endl;
        LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; UpdateDevices" << endl;

        m_ArenaSDKSystem->UpdateDevices(DEFAULT_WAIT_TIME);

        LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; GetDevices";

        std::vector<Arena::DeviceInfo> deviceInfos = m_ArenaSDKSystem->GetDevices();
     
        LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; Found " << deviceInfos.size() << " candidates devices" << endl;

        for (int i = 0; i< deviceInfos.size(); i++)
        {
            Arena::DeviceInfo& device_info = deviceInfos[i];

            gcstring name = device_info.ModelName();
            gcstring addr = device_info.IpAddressStr();
            gcstring serial = device_info.SerialNumber();

            std::string s_name = std::string(name);
            std::string s_addr = std::string(addr);
            std::string s_serial = std::string(serial);

            LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; #" <<i << " " << s_name << endl;

            if ( s_name.find ( "PHX016S" ) != std::string::npos || s_name.find("PHX032S") != std::string::npos)
            {
                LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; Phoenix camera found" << endl;

                CameraDescription c;

                c.Description = "NAME[" + s_name + "] SDK[ARENASDK] IP[" + s_addr + "]";
                c.Address = s_addr;
                c.Interface = 0;
                c.Serial = s_serial;
                c.Sdk = CamSdkType::LUCID_ARENA;
                c.DeviceType = InputDeviceType::CAMERA;

                Devices.push_back(c);
            }
        }
    }
    catch (std::exception& e)
    {
        LOG_ERROR << "CameraLucidArena_Scanner::UpdateCameraList" << "Arena::OpenSystem failure " << e.what() << endl;
    }
}
