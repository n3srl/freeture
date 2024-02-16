#pragma once

//headers refactoring ok
#include "Commons.h"

#include <vector>
#include <memory>
#include <mutex>

#include "ECamSdkType.h"
#include "SParam.h"

namespace freeture
{
    class Device;
    class Camera;
    class CameraDescription;

    class CameraDeviceManager
    {
    public:
          CameraDeviceManager();

    private:
        static std::shared_ptr<CameraDeviceManager> m_Instance;
        static std::mutex m_Mutex;

        std::vector<CameraDescription> m_DevicesList;
        Device* m_Device;
        Camera* m_Camera;

        parameters m_SelectedRuntimeConfiguration;
        unsigned short m_SelectedDeviceID = 0;
        CamSdkType m_SelectedSdk;

        bool m_Selected = false;
        int m_DeviceCount;

    public:
        CameraDeviceManager(const CameraDeviceManager&) = default;
        virtual ~CameraDeviceManager() = default;

        static std::shared_ptr<CameraDeviceManager> Get()
        {
            std::lock_guard<std::mutex> lock(m_Mutex);

            if (!m_Instance)
                m_Instance = std::make_shared<CameraDeviceManager>();

            return m_Instance;
        }

        bool selectDevice(parameters);
        Device* getDevice();

        void printDevicesList();

      private:
        int getCameraDeviceBySerial(std::string);

        std::vector<CameraDescription> getListDevice();

        /// <summary>
        /// Create the camera for the selected device
        /// </summary>
        /// <returns></returns>
        bool createCamera();

        /// <summary>
        /// Create the device with the camera
        /// </summary>
        /// <returns></returns>
        bool createDevice();

        //da rimuovere
        CamSdkType getDeviceSdk(int id);

        void mergeList(std::vector<CameraDescription>&);

    };
}