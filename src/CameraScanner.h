#pragma once
/*
    THIS CLASS IMPLEMENTS:
        - ABSTRACT FACTORY
        - FACTORY METHOD
*/

//header refactoring ok
#include "Commons.h"

#include <string>
#include <vector>

#include "EInputDeviceType.h"
#include "ECamSdkType.h"
#include "ECamModel.h"

namespace freeture
{
    class CameraDescription
    {
    public:
        int Id = 0;
        int Interface = 0;

        std::string Description = "";
        int DeviceId = 0;
        std::string Address = "";
        std::string Serial = "";

        InputDeviceType DeviceType = InputDeviceType::UNDEFINED_INPUT_TYPE;
        CamSdkType Sdk = CamSdkType::UNKNOWN;
        CamModel Model = CamModel::NONE;
    };

    /*STRATEGY*/
    class CameraScanner
    {
    protected:
        virtual void UpdateCameraList() = 0;

    public:
        CamSdkType Sdk;

        std::vector<CameraDescription> Devices;

        CameraScanner() = delete;

        CameraScanner(CamSdkType);

        virtual ~CameraScanner();

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
}