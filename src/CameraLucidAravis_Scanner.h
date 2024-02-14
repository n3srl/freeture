#pragma once

/**
* \file    CameraLucidArena.h
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief   Use Aravis library to pilot Lucid phoenix cameras.
*
*/

#ifdef LINUX
#include "Commons.h"
#include "CameraDeviceManager.h"

    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>

    #include <iostream>
    #include <string>
    #include "Frame.h"
    #include "TimeDate.h"
    #include "Camera.h"
    #include "arv.h"
    #include "arvinterface.h"
    #include <time.h>
    #include <algorithm>
    #include "EParser.h"

    #define BOOST_LOG_DYN_LINK 1

    #include <boost/log/common.hpp>
    #include <boost/log/expressions.hpp>
    #include <boost/log/utility/setup/file.hpp>
    #include <boost/log/utility/setup/console.hpp>
    #include <boost/log/utility/setup/common_attributes.hpp>
    #include <boost/log/attributes/named_scope.hpp>
    #include <boost/log/attributes.hpp>
    #include <boost/log/sinks.hpp>
    #include <boost/log/sources/logger.hpp>
    #include <boost/log/core.hpp>
    #include "ELogSeverityLevel.h"

namespace freeture
{
    class CameraLucidAravis_Scanner : public CameraScanner
    {

    public:
        virtual void UpdateCameraList() override
        {
            std::cout << "GenericAravisCameraScanner::UpdateCameraList" << std::endl;
            ArvInterface* interface;

            //arv_update_device_list();

            int ni = arv_get_n_interfaces();


            for (int j = 0; j < ni; j++)
            {

                const char* name = arv_get_interface_id(j);

                if (strcmp(name, "GigEVision") == 0) {
                    interface = arv_gv_interface_get_instance();
                    arv_interface_update_device_list(interface);
                    //int nb = arv_get_n_devices();

                    int nb = arv_interface_get_n_devices(interface);

                    for (int i = 0; i < nb; i++) {

                        CameraDescription c;

                        //const char* str = arv_get_device_id(i);
                        const char* str = arv_interface_get_device_id(interface, i);
                        const char* addr = arv_interface_get_device_address(interface, i);
                        std::string s = str;
                        std::string t = "LUCID";
                        if (s.find(t) != std::string::npos)
                        {
                            c.Id = i;
                            c.Description = "NAME[" + s + "] SDK[ARAVIS] IP: " + addr;
                            c.DeviceId = "" + s;
                            c.Address = std::string(addr);
                            c.Interface = j;
                            c.Sdk = CamSdkType::ARAVIS;

                            Devices.push_back(c);
                        }
                    }
                }
            }
        }

    };
}
#endif
