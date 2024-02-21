#pragma once
#include "Commons.h"

#ifdef LINUX

#include "CameraScanner.h"

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
