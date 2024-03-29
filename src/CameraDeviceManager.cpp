#include "CameraDeviceManager.h"

#include "Logger.h"
#include "Conversion.h"
#include "EParser.h"
#include "ECamModel.h"
#include "CameraFactory.h"
#include "Camera.h"
#include "Device.h"
#include "CameraScanner.h"

using namespace freeture;

using namespace std;

shared_ptr<CameraDeviceManager> CameraDeviceManager::m_Instance = nullptr;
mutex CameraDeviceManager::m_Mutex;

CameraDeviceManager::CameraDeviceManager() 
{
    LOG_DEBUG << "CameraDeviceManager::CameraDeviceManager;" <<"Initialize available scanners...";

    initScanners();

    LOG_DEBUG << "CameraDeviceManager::CameraDeviceManager;" << "Retrieve devices list...";

    m_DeviceCount = -1;

    m_DevicesList = getListDevice();

    m_DeviceCount = m_DevicesList.size();
}

bool CameraDeviceManager::createDevice()
{
    LOG_DEBUG << "CameraDeviceManager::createDevice";
    m_Device = new Device();

    //assign camera
    m_Device->setCamera( m_Camera );
    
    //Setup with runtime values
    m_Device->Setup(m_SelectedRuntimeConfiguration.camInput, m_SelectedRuntimeConfiguration.framesInput, m_SelectedRuntimeConfiguration.vidInput);

    return  true;
}

bool CameraDeviceManager::createCamera()
{
    LOG_DEBUG << "CameraDeviceManager::createCamera";
    
    if (m_SelectedRuntimeConfiguration.DEVICE_ID >= 0 && m_SelectedRuntimeConfiguration.DEVICE_ID <= m_DeviceCount - 1)
    {
        CameraDescription camera_description = m_DevicesList.at(m_SelectedRuntimeConfiguration.DEVICE_ID);

        LOG_DEBUG << "CREATE CAMERA " + camera_description.Description;
        // Create Camera object with the correct sdk.

        m_Camera = CameraFactory::createCamera(camera_description, m_SelectedRuntimeConfiguration);

        if (m_Camera == nullptr)
        {
            LOG_DEBUG << "Fail to select correct sdk" ;
            EParser<CamSdkType> parser;
            LOG_DEBUG << "Fail to select correct sdk : " << parser.getStringEnum(camera_description.Sdk) ;
            return false;
        }

        return true;
    }

    LOG_ERROR << "No device with ID " << m_SelectedRuntimeConfiguration.DEVICE_ID;

    return false;
}

Device* CameraDeviceManager::getDevice() {
    LOG_DEBUG << "CameraDeviceManager::getDevice";
    return m_Device;
}

int CameraDeviceManager::getCameraDeviceBySerial(string serial)
{
    LOG_DEBUG << "CameraDeviceManager::getCameraDeviceBySerial";
    if(serial == "") {
        LOG_DEBUG << "CameraDeviceManager::getCameraDeviceBySerial;"<< "CAMERA SERIAL IS MISSING DEVICE ID WILL BE USED " ;
        return -1;
    }
    LOG_DEBUG << "CameraDeviceManager::getCameraDeviceBySerial;" << "LOOKING FOR CAMERA SERIAL " << serial ;
    for (int i = 0; i < m_DevicesList.size(); i++)
    {
        CameraDescription cameraDescrption = m_DevicesList.at(i);
        size_t found = cameraDescrption.Description.find(serial);

        if(found != string::npos) return i;
    }
    return -2;
}


bool CameraDeviceManager::selectDevice(parameters runtime_configuration)
{
    if (m_Selected)
    {
        if (runtime_configuration.DEVICE_ID == m_SelectedDeviceID)
            return true;
        else
        {
            LOG_ERROR << "Device #" << m_SelectedDeviceID << " is already selected.";
            return false;
        }
    }

    if (m_DeviceCount < 0 || m_SelectedDeviceID >(m_DeviceCount - 1)) {
        LOG_ERROR << "CameraDeviceManager::selectDevice" << "CAMERA_ID's value not exist.";
        m_Selected = false;
        return false;
    }

    m_Selected = true;
    m_SelectedRuntimeConfiguration = runtime_configuration;
    m_SelectedDeviceID = runtime_configuration.DEVICE_ID;
    m_SelectedSdk = getDeviceSdk(runtime_configuration.DEVICE_ID);


    //create camera
    if (!createCamera()) {
        LOG_ERROR << " CameraDeviceManager::selectDevice" << "Error creating camera";
        m_Selected = false;
        return false;
    }

    //create device
    if (!createDevice()) {
        LOG_ERROR << " CameraDeviceManager::selectDevice" << "Error creating device";
        m_Selected = false;
        return false;
    }

    return true;
}

CamSdkType CameraDeviceManager::getDeviceSdk(int id) {
    shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
    vector<CameraDescription> devices = m_CameraDeviceManager->getListDevice();

    if (id >= 0 && id <= (m_CameraDeviceManager->m_DeviceCount - 1)) {
        return devices.at(id).Sdk;
    }
    LOG_DEBUG << "ERROR GETTING DEVICE SDK FOR CAMERA ID " << id << " NUM OF DEVICES " << (m_CameraDeviceManager->m_DeviceCount - 1) ;

    return CamSdkType::UNKNOWN;
}

vector<CameraDescription> CameraDeviceManager::getListDevice()
{
    LOG_DEBUG << "CameraDeviceManager::getListDevice";

    for (scanner_type scanner : m_AvailableScanners) 
    {
        mergeList(scanner->getCamerasList());
    }

    //enumerate devices
    for (int id = 0; id < m_DevicesList.size(); id++)
        m_DevicesList[id].Id = id;

    return m_DevicesList;
}



void CameraDeviceManager::mergeList(vector<CameraDescription>& Devices)
{
    //foreach device found test if already exists. if not add to list
    for (int i = 0; i < Devices.size(); i++)
    {
        bool add_to_list = true;

        for (int j = 0; j < m_DevicesList.size(); j++)
            if (m_DevicesList[j].Address == Devices[i].Address && m_DevicesList[j].Sdk == Devices[i].Sdk)
            {
                add_to_list = false;
                break;
            }

        if (add_to_list)
            m_DevicesList.push_back(Devices[i]);
    }
}

void CameraDeviceManager::printDevicesList()
{
    LOG_DEBUG << "CameraDeviceManager::printDevicesList";

    for (int i = 0; i < m_DevicesList.size(); i++)
    {
        CameraDescription cam = m_DevicesList[i];
        LOG_INFO << "[" << cam.Id << "] - " << cam.Address << "  " << cam.Description;
    }
}

void CameraDeviceManager::initScanners()
{
    LOG_DEBUG << "CameraDeviceManager::initScanners";

    scanner_type ptr;

    // PYLONGIGE
#ifdef USE_PYLON
    LOG_INFO << "PYLON SCANNER AVAILABLE";
    ptr = CameraScanner::CreateScanner(CamSdkType::PYLONGIGE);
    assert(ptr != nullptr);
    m_AvailableScanners.push_back(ptr);
#endif

#ifdef TISCAMERA
    LOG_INFO << "TIS CAMERA SCANNER AVAILABLE";
    ptr = CameraScanner::CreateScanner(CamSdkType::TIS);
    assert(ptr != nullptr);
    m_AvailableScanners.push_back(ptr);
#endif

#ifdef VIDEOINPUT
    LOG_INFO << "VIDEO INPUT SCANNER AVAILABLE";
    ptr = CameraScanner::CreateScanner(CamSdkType::VIDEOINPUT);
    assert(ptr != nullptr);
    m_AvailableScanners.push_back(ptr);
#endif

#ifdef USE_ARAVIS
    LOG_INFO << "LUCID ARAVIS SCANNER AVAILABLE";
    ptr = CameraScanner::CreateScanner(CamSdkType::LUCID_ARAVIS);
    assert(ptr != nullptr);
    m_AvailableScanners.push_back(ptr);
#endif

#ifdef USE_ARENA
    LOG_INFO << "LUCID ARENA SCANNER AVAILABLE";
    ptr =  CameraScanner::CreateScanner(CamSdkType::LUCID_ARENA);
    assert(ptr != nullptr);
    m_AvailableScanners.push_back(ptr);
#endif

#ifdef USE_ARAVIS
    // ARAVIS
    LOG_INFO << "ARAVIS SCANNER AVAILABLE";
    ptr = CameraScanner::CreateScanner(CamSdkType::ARAVIS);
    assert(ptr != nullptr);
    m_AvailableScanners.push_back(ptr);
#endif

/*
* DA SVILUPPARE!
* 
 LOG_INFO << "VIDEO FILE SCANNER AVAILABLE";
ptr = CameraScanner::CreateScanner(CamSdkType::VIDEOFILE);
assert(ptr != nullptr);
m_AvailableScanners.push_back(ptr);
*/
}

size_t CameraDeviceManager::getDeviceCount()
{
    return m_DeviceCount;

}
