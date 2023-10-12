#include "CameraDeviceManager.h"



CameraDeviceManager::CameraDeviceManager() {
    std::cout << "INIT CAMERADEVICE MANAGER " << std::endl;
    deviceNumber = -1;
    dev = freeture::Device();
    deviceList = dev.getListDevice();
    deviceNumber = deviceList.size();
    
};



// Getters
std::vector<CameraDescription> CameraDeviceManager::getListDevice() {
    return deviceList;
}

freeture::Device* CameraDeviceManager::getDevice() {
    return &dev;
}

void CameraDeviceManager::listDevice() {
    for(int i = 0; i < deviceList.size(); i++)
    {
        CameraDescription cam = deviceList[i];
        cam.Id = i;
        std::cout << "[" << cam.Id << "]    " << cam.Description << std::endl;
    }
}
