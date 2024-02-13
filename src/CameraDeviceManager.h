#pragma once

//headers refactoring ok
#include "Commons.h"

#include <vector>

#include "CameraScanner.h"
#include "Device.h"

namespace freeture
{
    class CameraDeviceManager
    {
    private:

        std::vector<CameraDescription> deviceList;
        Device dev;
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

        Device* getDevice();

        void listDevice(bool);
        int getCameraDeviceBySerial(std::string);
    };
}