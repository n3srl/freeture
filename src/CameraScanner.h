#pragma once

#include <string>
#include <vector>

#include "ECamSdkType.h"

class CameraDescription
{
    public:
        int Id                      = 0;
        std::string Description     = "";
        std::string DeviceId        = "";
        std::string Address         = "";
        int Interface               = 0;
        CamSdkType Sdk              = CamSdkType::UNKNOWN;
};

/*STRATEGY*/
class CameraScanner
{
    protected:
            virtual void UpdateCameraList()=0;

    public:
        CamSdkType Sdk;

        CameraScanner() = delete;

        CameraScanner(CamSdkType sdk):Sdk(sdk)
        {
        }

        std::vector<CameraDescription> Devices;


        /**
            Update the List of camera descriptions and return back the vector
         */
        std::vector<CameraDescription>& getCamerasList();

        /**
         * Scanneer factory
         */
        static CameraScanner* CreateScanner(CamSdkType sdk);

       /**
        * Print a list with connected devices.
        *
        */
        bool listCameras();
};
