#pragma once
#include "Commons.h"

#include "CameraScanner.h"

#ifdef LINUX

namespace freeture
{
    class CameraGigeAravis_Scanner : public CameraScanner
    {
    public:
        CameraGigeAravis_Scanner(CamSdkType);

        virtual void UpdateCameraList() override;

    };
}
#endif
