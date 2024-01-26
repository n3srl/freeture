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

void CameraDeviceManager::listDevice(bool print = true) {
    for(int i = 0; i < deviceList.size(); i++)
    {
        CameraDescription cam = deviceList[i];
        cam.Id = i;
        if(print) std::cout << "[" << cam.Id << "]    " << cam.Description << std::endl;
    }
}

int CameraDeviceManager::getCameraDeviceBySerial(std::string serial)
{
    if(serial == "") {
        std::cout << "CAMERA SERIAL IS MISSING DEVICE ID WILL BE USED " << std::endl;
        return -1;
    }
    std::cout << "LOOKING FOR CAMERA SERIAL " << serial << std::endl;
    for (int i = 0; i < deviceList.size(); i++)
    {
        CameraDescription cameraDescrption = deviceList.at(i);
        size_t found = cameraDescrption.Description.find(serial);

        if(found != std::string::npos) return i;
    }
    return -2;
}