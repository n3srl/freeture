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

CameraDeviceManager::CameraDeviceManager() {
    LOG_DEBUG << "CameraDeviceManager::CameraDeviceManager";

    LOG_INFO << "Retrieve devices list...";

    m_DeviceCount = -1;

    m_DevicesList = getListDevice();

    m_DeviceCount = m_DevicesList.size();

    if (m_DeviceCount!=1)
        LOG_INFO << "Found "<< m_DeviceCount << " devices!";
    else
        LOG_INFO << "Found " << m_DeviceCount << " device!";
}

bool CameraDeviceManager::createDevice()
{
    LOG_DEBUG << "CameraDeviceManager::createDevice";
    m_Device = new Device();

    //assign camera
    m_Device->mCam = m_Camera;
    
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
    if (m_Selected) {
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
}

vector<CameraDescription> CameraDeviceManager::getListDevice()
{
    LOG_DEBUG << "CameraDeviceManager::getListDevice";

#ifdef WINDOWS

    // PYLONGIGE
#ifdef USE_PYLON
//         mCam = new CameraGigePylon();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
//             elem.first = mNbDev; elem.second = PYLONGIGE;
//             subElem.first = listCams.at(i).first; subElem.second = elem;
//             mDevices.push_back(subElem);
//             if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second ;
//             mNbDev++;
//         }
//         delete mCam;
#endif
#ifdef TISCAMERA
//         // TIS
// 
//         mCam = new CameraGigeTis();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
//             elem.first = mNbDev; elem.second = TIS;
//             subElem.first = listCams.at(i).first; subElem.second = elem;
//             mDevices.push_back(subElem);
//             if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second ;
//             mNbDev++;
//         }
//         delete mCam;
#endif
#ifdef VIDEOINPUT
//         // WINDOWS
// 
//         mCam = new CameraWindows();
//         listCams = mCam->getCamerasList();
//         for(int i = 0; i < listCams.size(); i++) {
// 
//             // Avoid to list basler
//             string::size_type pos1 = listCams.at(i).second.find("Basler");
//             string::size_type pos2 = listCams.at(i).second.find("BASLER");
//             if((pos1 != string::npos) || (pos2 != string::npos)) {
//                 //LOG_DEBUG << "found \"words\" at position " << pos1 ;
//             } else {
//                 elem.first = mNbDev; elem.second = VIDEOINPUT;
//                 subElem.first = listCams.at(i).first; subElem.second = elem;
//                 mDevices.push_back(subElem);
//                 if(printInfos) cout << "[" << mNbDev << "]    " << listCams.at(i).second ;
//                 mNbDev++;
//             }
//         }
// 
//         delete mCam;
#endif
#endif


#ifdef USE_ARAVIS
    CameraScanner* lucid_aravis_scanner = Camera::Scanner->CreateScanner(CamSdkType::LUCID_ARAVIS);
    assert(lucid_aravis_scanner != nullptr);
    lucid_aravis_scanner->getCamerasList();
    listCams.insert(listCams.end(), lucid_aravis_scanner->Devices.begin(), lucid_aravis_scanner->Devices.end());
#endif

#ifdef USE_ARENA
    //ARENA SDK
    CameraScanner* lucid_arena_scanner = Camera::Scanner->CreateScanner(CamSdkType::LUCID_ARENA);

    assert(lucid_arena_scanner != nullptr);
   
    mergeList(lucid_arena_scanner->getCamerasList());
#endif

#ifdef USE_ARAVIS
    // ARAVIS
    CameraScanner* aravis_scanner = Camera::Scanner->CreateScanner(CamSdkType::ARAVIS);
    assert(aravis_scanner != nullptr);
    aravis_scanner->getCamerasList();

    mergeList(aravis_scanner->Devices);
#endif

    // VIDEO
    /*
    elem.first = mNbDev; elem.second = VIDEOFILE;
    subElem.first = 0; subElem.second = elem;
    mDevices.push_back(subElem);
    if(printInfos) cout << "[" << mNbDev << "]    VIDEO FILES" ;
    mNbDev++;

    // FRAMES

    elem.first = mNbDev; elem.second = FRAMESDIR;
    subElem.first = 0; subElem.second = elem;
    mDevices.push_back(subElem);
    if(printInfos) cout << "[" << mNbDev << "]    FRAMES DIRECTORY" ;
    mNbDev++;
    */
    

    //enumerate devices
    for (int i = 0; i < m_DevicesList.size(); i++)
        m_DevicesList[i].Id = i;


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
