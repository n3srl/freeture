#include <iostream>

#include "CameraLucidArenaScanner.h"

#include "arv.h"
#include "arvinterface.h"

using namespace std;

CameraLucidArenaScanner::CameraLucidArenaScanner(CamSdkType sdk):CameraScanner(sdk)
{
}

            void CameraLucidArenaScanner::UpdateCameraList()
            {
                cout << "CameraLucidArenaScanner::UpdateCameraList" << endl;
                    ArvInterface *interface;

                    //arv_update_device_list();

                    int ni = arv_get_n_interfaces();


                    for (int j = 0; j< ni; j++)
                    {

                        const char* name = arv_get_interface_id (j);
                        
                        if (strcmp(name,"GigEVision") == 0) {
                            interface = arv_gv_interface_get_instance();
                            arv_interface_update_device_list(interface);
                            //int nb = arv_get_n_devices();

                            int nb = arv_interface_get_n_devices(interface);

                            for(int i = 0; i < nb; i++){
                                
                                CameraDescription c;

                                //const char* str = arv_get_device_id(i);
                                const char* str = arv_interface_get_device_id(interface,i);
                                const char* addr = arv_interface_get_device_address(interface,i);
                                string s = str;
                                string t = "Lucid";
                                if (s.find(t)!= string::npos)
                                {
                                    
                                    c.Id = i;
                                    c.Description = "NAME[" + s + "] SDK[LUCIDARAVIS] IP: " + addr;
                                    c.DeviceId =string(str);
                                    c.Address = string(addr);
                                    c.Interface = j;
                                    c.Sdk = CamSdkType::LUCID_ARAVIS;

                                    Devices.push_back(c);
                                }
                                t = "Machine";
                                if (s.find(t)!= string::npos)
                                {
                                    
                                    c.Id = i;
                                    c.Description = "NAME[" + s + "] SDK[LUCIDARAVIS] IP: " + addr;
                                    c.DeviceId =string(str);
                                    c.Address = string(addr);
                                    c.Interface = j;
                                    c.Sdk = CamSdkType::LUCID_ARAVIS;

                                    Devices.push_back(c);
                                }
                            }
                        }
                    }
            }

