#include "CameraLucidArena_Scanner.h"

#include <iostream>

#include <SaveApi.h>
#include <ArenaApi.h>
#include <GenICam.h>

#include "ErrorManager.cpp"
#include "Logger.h"

using namespace GenICam;
using namespace freeture;

CameraLucidArena_Scanner::CameraLucidArena_Scanner(CamSdkType sdk) : CameraScanner(sdk)
{
}
CameraLucidArena_Scanner::~CameraLucidArena_Scanner()
{
    if (m_ArenaSDKSystem == nullptr)
        Arena::CloseSystem(m_ArenaSDKSystem);
}

/**
 * Called to get the current reachable cameras.
 * Lucid cameras are GigEVision cameras and a specific tag into the name. "PHX016S"
 */
void CameraLucidArena_Scanner::UpdateCameraList()
{
    LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList";

    try
    {
        if (m_ArenaSDKSystem == nullptr)
            m_ArenaSDKSystem =  Arena::OpenSystem();
        
        
        
        if (m_ArenaSDKSystem == nullptr) {
            LOG_ERROR << "CameraLucidArena_Scanner::UpdateCameraList; Arena::OpenSystem returned nullptr";
            throw std::runtime_error("Arena::OpenSystem returned nullptr");
        }
        LOG_INFO  << "CameraLucidArena_Scanner::UpdateCameraList; Arena SDK Version: " <<Arena::GetVersion().c_str();
        LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; UpdateDevices";

        //arv_update_device_list();
        m_ArenaSDKSystem->UpdateDevices(1000);
        LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; GetDevices";

        std::vector<Arena::DeviceInfo> deviceInfos = m_ArenaSDKSystem->GetDevices();
     
        int ni =  deviceInfos.size();
        LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; Found " << ni << " candidates devices";

        for (int i = 0; i< ni; i++)
        {
            Arena::DeviceInfo& device_info = deviceInfos[i];

            gcstring name = device_info.ModelName();
            gcstring addr = device_info.IpAddressStr();

            std::string s_name = std::string(name);
            std::string s_addr = std::string(addr);

            LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; #" <<i << " " << s_name;

            if ( s_name.find ( "PHX016S" ) != std::string::npos || s_name.find("PHX032S") != std::string::npos)
            {
                LOG_DEBUG << "CameraLucidArena_Scanner::UpdateCameraList; Phoenix camera found found";

                CameraDescription c;

                //const char* str = arv_get_device_id(i);
                //std::cout << "FIND " + s_name << std::endl;
                c.Id = i;
                c.Description = "NAME[" + s_name + "] SDK[ARENASDK] IP: " + s_addr;
                c.DeviceId = s_name;
                c.Address = s_addr;
                c.Interface = 0;
                c.Sdk = CamSdkType::LUCID_ARENA;
                Devices.push_back(c);
            }
        }
    }
    catch (std::exception& e)
    {
        LOG_DEBUG << ("CameraLucidArena_Scanner::UpdateCameraList","Arena::OpenSystem failure ",e.what());
        ErrorManager::Exception(e);
        return;
    }

}
