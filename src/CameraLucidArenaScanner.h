#pragma once
#include "Commons.h"

#include "CameraScanner.h"

namespace freeture
{
    class CameraLucidArenaScanner : public CameraScanner
    {

    public:
        CameraLucidArenaScanner(CamSdkType);

        virtual void UpdateCameraList() override;

    };
}