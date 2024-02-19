#pragma once
#include "Commons.h"

#ifdef LINUX

#include "CameraScanner.h"

namespace freeture
{
    class CameraGigeAravisScanner : public CameraScanner
    {
    public:
        CameraGigeAravisScanner(CamSdkType);

        virtual void UpdateCameraList() override;

    };
}
#endif
