#pragma once

#include "CameraScanner.h"

class CameraGigeAravisScanner: public CameraScanner
{
    public:
       CameraGigeAravisScanner(CamSdkType);

       virtual void UpdateCameraList() override;

};
