#pragma once

#include "CameraScanner.h"
#include "Device.h"

using namespace freeture;

class CameraDeviceManager
{
    private:

        std::vector<CameraDescription> deviceList;
        freeture::Device dev;
        CameraDeviceManager();



    public:
        int deviceNumber;
         
        CameraDeviceManager(CameraDeviceManager&) = delete;

        static CameraDeviceManager& Get()
        {
            static CameraDeviceManager instance;
            return instance;
        }

        // Getters
        std::vector<CameraDescription> getListDevice();

        freeture::Device* getDevice();

        void listDevice();


};