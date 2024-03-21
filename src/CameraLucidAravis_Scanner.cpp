#include "CameraLucidAravis_Scanner.h"
#include <string>

#include "Logger.h"

#ifdef LINUX
#ifdef USE_ARAVIS

#include "arv.h"
#include "arvinterface.h"


using namespace std;
using namespace freeture;

CameraLucidAravis_Scanner::CameraLucidAravis_Scanner(CamSdkType sdk): CameraScanner(sdk)
{
    
}

CameraLucidAravis_Scanner::~CameraLucidAravis_Scanner()
{

}

void CameraLucidAravis_Scanner::UpdateCameraList()
{
    LOG_DEBUG << "CameraLucidAravis_Scanner::UpdateCameraList" << endl;
    ArvInterface* interface;

    //arv_update_device_list();

    int ni = arv_get_n_interfaces();

    for (int j = 0; j < ni; j++)
    {
        const char* name = arv_get_interface_id(j);

        if (strcmp(name, "GigEVision") == 0)
        {
            interface = arv_gv_interface_get_instance();
            arv_interface_update_device_list(interface);
            //int nb = arv_get_n_devices();

            int nb = arv_interface_get_n_devices(interface);
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

                c.Description = "NAME[" + s + "] SDK[LUCIDARAVIS] IP[" + addr + "]";
                c.Address = string(addr);
                c.Interface = j;
                c.Serial = serialNumber;
                c.Sdk = CamSdkType::LUCID_ARAVIS;
                c.DeviceType = InputDeviceType::CAMERA;

                Devices.push_back(c);
            }
        }
    }
}

#endif
#endif
