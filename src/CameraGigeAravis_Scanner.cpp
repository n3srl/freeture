/**
* \file    CameraGigeAravis_Scanner.cpp
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.0
* \date    02/21/2024
* \brief   Use Aravis SDK library to scan Gige Cameras
*
*/
#include "CameraGigeAravis_Scanner.h"

#include "Logger.h"

#include <iostream>

#ifdef LINUX
#ifdef USE_ARAVIS

#include "arv.h"
#include "arvinterface.h"

using namespace std;
using namespace freeture;

CameraGigeAravis_Scanner::CameraGigeAravis_Scanner(CamSdkType sdk) : CameraScanner(sdk)
{
}

CameraGigeAravis_Scanner::~CameraGigeAravis_Scanner()
{

}

void CameraGigeAravis_Scanner::UpdateCameraList()
{
    LOG_DEBUG << "CameraGigeAravis_Scanner::UpdateCameraList"<<endl;

    ArvInterface* interface;

    //arv_update_device_list();

    int ni = arv_get_n_interfaces();

    for (int j = 0; j < ni; j++)
    {
        const char* name = arv_get_interface_id(j);
        LOG_DEBUG << "CameraGigeAravis_Scanner::UpdateCameraList" << name << endl;

        if (strcmp(name, "GigEVision") == 0)
        {

            interface = arv_gv_interface_get_instance();
            arv_interface_update_device_list(interface);
            //int nb = arv_get_n_devices();

            int nb = arv_interface_get_n_devices(interface);
            LOG_DEBUG << "CameraGigeAravis_Scanner::UpdateCameraList" << "Devices detected: " << nb << endl;
            std::string delimiter = "-";

            for (int i = 0; i < nb; i++)
            {
                CameraDescription c;

               
                //const char* str = arv_get_device_id(i);
                const char* str = arv_interface_get_device_id(interface, i);
                const char* addr = arv_interface_get_device_address(interface, i);

                string s = str;
                string a = addr;

                //extract serial number from name
                size_t pos = s.rfind(delimiter);
                string serialNumber = s.substr(pos + 1);

                c.Description = "NAME[" + s + "] SDK[ARAVIS] IP[ " + a + "]";

                c.Address = string(addr);
                c.Interface = j;
                c.Serial = serialNumber;
                c.Sdk = CamSdkType::ARAVIS;
                c.DeviceType = InputDeviceType::CAMERA;

                Devices.push_back(c);
            }
        }
    }
}


#endif
#endif
