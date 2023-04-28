#include <iostream>

#include <SaveApi.h>

#include "ErrorManager.cpp"
#include "CameraLucidArena_PHX016SScanner.h"
#include "Logger.h"


CameraLucidArena_PHX016SScanner::CameraLucidArena_PHX016SScanner(CamSdkType sdk):CameraScanner(sdk)
{
}

/**
 * Called to get the current reachable cameras.
 * Lucid cameras are GigEVision cameras and a specific tag into the name. "PHX016S"
 */
void CameraLucidArena_PHX016SScanner::UpdateCameraList()
{
    freeture::LogDebug("CameraLucidArena_PHX016SScanner::UpdateCameraList");

    try
    {
        if (m_ArenaSDKSystem == nullptr)
            m_ArenaSDKSystem =  Arena::OpenSystem();

        if (m_ArenaSDKSystem == nullptr)
            throw std::runtime_error("Arena::OpenSystem returned nullptr");

        freeture::LogDebug("CameraLucidArena_PHX016SScanner::UpdateCameraList","UpdateDevices");

        //arv_update_device_list();
        m_ArenaSDKSystem->UpdateDevices(100);
        freeture::LogDebug("CameraLucidArena_PHX016SScanner::UpdateCameraList","GetDevices");

        std::vector<Arena::DeviceInfo> deviceInfos = m_ArenaSDKSystem->GetDevices();

        int ni =  deviceInfos.size();
        freeture::LogDebug("CameraLucidArena_PHX016SScanner::UpdateCameraList","Found ",ni, "candidates devices");

        for (int i = 0; i< ni; i++)
        {
            Arena::DeviceInfo& device_info = deviceInfos[i];
            const char* name = device_info.ModelName();

            freeture::LogDebug("CameraLucidArena_PHX016SScanner::UpdateCameraList","#",i," ",name);
            std::string s_name = std::string(name);
            std::string s_test = std::string("PHX016S");
            if ( s_name.find ( s_test ) != std::string::npos )
            {
                freeture::LogDebug("CameraLucidArena_PHX016SScanner::UpdateCameraList","PHX016S found");

                CameraDescription c;

                //const char* str = arv_get_device_id(i);
                const char* str = name;
                const char* addr = device_info.IpAddressStr();
                std::string s = s_name;
                //std::cout << "FIND " + s_name << std::endl;
                c.Id = i;
                c.Description = "NAME[" + s + "] SDK[ARENASDK] IP: " + addr;
                c.DeviceId =std::string(name);
                c.Address = std::string(addr);
                c.Interface = 0;
                c.Sdk = CamSdkType::LUCID_ARENA;
                Devices.push_back(c);
            }
        }
    }
    catch (std::exception& e)
    {
        freeture::LogError("CameraLucidArena_PHX016SScanner::UpdateCameraList","Arena::OpenSystem failure ",e.what());
        ErrorManager::Exception(e);
        return;
    }

}
