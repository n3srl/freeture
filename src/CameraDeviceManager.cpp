#include "CameraDeviceManager.h"

#include "Logger.h"
#include "Conversion.h"
#include "EParser.h"

using namespace freeture;

using namespace std;

shared_ptr<CameraDeviceManager> CameraDeviceManager::m_Instance = nullptr;
mutex CameraDeviceManager::m_Mutex;

CameraDeviceManager::CameraDeviceManager() {
    LOG_DEBUG << "CameraDeviceManager::CameraDeviceManager";

    LOG_INFO << "Retrieve devices list...";

    deviceCount = -1;
    dev = Device();
    deviceList = dev.getListDevice();
    deviceCount = deviceList.size();
    LOG_INFO << "Found "<< deviceCount << " devices";
}

// Getters
vector<CameraDescription> CameraDeviceManager::getListDevice() {
    LOG_DEBUG << "CameraDeviceManager::getListDevice";
    return deviceList;
}

freeture::Device* CameraDeviceManager::getDevice() {
    LOG_DEBUG << "CameraDeviceManager::getDevice";
    return &dev;
}

void CameraDeviceManager::listDevice(bool print = true) {
    LOG_DEBUG << "CameraDeviceManager::listDevice";

    for(int i = 0; i < deviceList.size(); i++)
    {
        CameraDescription cam = deviceList[i];
        cam.Id = i;
        if(print)
            LOG_DEBUG << "CameraDeviceManager::listDevice;" << "[" << cam.Id << "]    " << cam.Description << endl;
    }
}

int CameraDeviceManager::getCameraDeviceBySerial(string serial)
{
    LOG_DEBUG << "CameraDeviceManager::getCameraDeviceBySerial";
    if(serial == "") {
        LOG_DEBUG << "CameraDeviceManager::getCameraDeviceBySerial;"<< "CAMERA SERIAL IS MISSING DEVICE ID WILL BE USED " << endl;
        return -1;
    }
    LOG_DEBUG << "CameraDeviceManager::getCameraDeviceBySerial;" << "LOOKING FOR CAMERA SERIAL " << serial << endl;
    for (int i = 0; i < deviceList.size(); i++)
    {
        CameraDescription cameraDescrption = deviceList.at(i);
        size_t found = cameraDescrption.Description.find(serial);

        if(found != string::npos) return i;
    }
    return -2;
}

bool CameraDeviceManager::selectDevice(unsigned short device_id)
{
    m_Selected = true;
    
    if (deviceCount < 0 || m_SelectedDevice >(deviceCount - 1)) {
        LOG_ERROR << "CameraDeviceManager::selectDevice" << "CAMERA_ID's value not exist.";
        return false;
    }

    m_SelectedDevice = device_id;
    if (!dev.createCamera()) {
        LOG_ERROR << " CameraDeviceManager::selectDevice" <<"Error creating camera";
        return false;
    }
    CamSdkType sdk = dev.getDeviceSdk(device_id);
    return true;
}

bool CameraDeviceManager::connect()
{
    //if first connection, then execute code
//     if (m_Param.CAMERA_INIT)
//     {
//         if (!device->firstIinitializeCamera(m_Param.CAMERA_INIT_CONFIG))
//         {
//             LOG_ERROR << "Inizializing camera using " << m_Param.CAMERA_INIT_CONFIG << " Failed!";
//             return;
//         }
//     }
    return false;
}

