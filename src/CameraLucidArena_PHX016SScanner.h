#pragma once

#include "CameraScanner.h"

#include <ArenaApi.h>

class CameraLucidArena_PHX016SScanner: public CameraScanner
{
    private:
        Arena::ISystem* m_ArenaSDKSystem = nullptr;

    public:
         CameraLucidArena_PHX016SScanner(CamSdkType);

        /**
         * Called to get the current reachable cameras.
         * Lucid cameras are GigEVision cameras and a specific tag into the name. "PHX016S"
         */
         virtual void UpdateCameraList() override;
};
