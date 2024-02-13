#pragma once
#ifdef LINUX
#include "Commons.h"

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
