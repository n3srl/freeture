#pragma once

//headers refactoring ok
#include "Commons.h"

#include <vector>
#include <memory>
#include <mutex>


#include "SParam.h"
#include "CfgParam.h"

#include "CameraScanner.h"
#include "Device.h"

namespace freeture
{
    class CameraDeviceManager
    {
    public:
          CameraDeviceManager();
// 

//          CameraDeviceManager& operator=(const CameraDeviceManager&) = delete;
//          CameraDeviceManager(CameraDeviceManager&&) = delete;
//          CameraDeviceManager& operator=(CameraDeviceManager&&) = delete;

    private:
      //  InputDeviceType m_InputType;

        static std::shared_ptr<CameraDeviceManager> m_Instance;
        static std::mutex m_Mutex;

        std::vector<CameraDescription> deviceList;
        Device dev;

        unsigned short m_SelectedDevice = 0;
        bool m_Selected = false;
    public:
        CameraDeviceManager(const CameraDeviceManager&) = default;
        int deviceCount;
        virtual ~CameraDeviceManager() = default;

        static std::shared_ptr<CameraDeviceManager> Get()
        {
            std::lock_guard<std::mutex> lock(m_Mutex);

            if (!m_Instance)
                m_Instance = std::make_shared<CameraDeviceManager>();

            return m_Instance;
        }

        bool selectDevice(unsigned short);

        // Getters
        std::vector<CameraDescription> getListDevice();
        Device* getDevice();
        void listDevice(bool);
        int getCameraDeviceBySerial(std::string);

        bool connect();

    };
}