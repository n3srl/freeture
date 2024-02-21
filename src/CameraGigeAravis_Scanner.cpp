#include "CameraGigeAravis_Scanner.h"

#include <iostream>

#include "Logger.h"

#ifdef LINUX

#include "arv.h"
#include "arvinterface.h"

using namespace std;
using namespace freeture;

CameraGigeAravis_Scanner::CameraGigeAravis_Scanner(CamSdkType sdk):CameraScanner(sdk)
{
}
        void CameraGigeAravis_Scanner::UpdateCameraList()
        {
            freeture::LogDebug("CameraGigeAravis_Scanner::UpdateCameraList");

            ArvInterface *interface;

            //arv_update_device_list();

            int ni = arv_get_n_interfaces();

            for (int j = 0; j< ni; j++)
            {
                const char* name = arv_get_interface_id (j);
                freeture::LogDebug("CameraGigeAravis_Scanner::UpdateCameraList",name);

                if (strcmp(name,"GigEVision") == 0)
                {
                    
                    interface = arv_gv_interface_get_instance();
                    arv_interface_update_device_list(interface);
                    //int nb = arv_get_n_devices();

                    int nb = arv_interface_get_n_devices(interface);
                    freeture::LogDebug("CameraGigeAravis_Scanner::UpdateCameraList","Devices detected: ", nb);

                    for(int i = 0; i < nb; i++)
                    {
                        CameraDescription c;
                        
                        //const char* str = arv_get_device_id(i);
                        const char* str = arv_interface_get_device_id(interface,i);
                        const char* addr = arv_interface_get_device_address(interface,i);

                        //freeture::LogDebug("CameraGigeAravis_Scanner::UpdateCameraList","\t- ", str," ",addr);

                        string s = str;
                        string a = addr;
                        
                        c.Id = i;
                        c.Description = "NAME[" + s + "] SDK[ARAVIS] IP: " + a;
                        c.DeviceId =string(str);
                        c.Address = string(addr);
                        c.Interface = j;
                        c.Sdk = CamSdkType::ARAVIS;
                        Devices.push_back(c);
                    }
                }
            }
        }



#endif
