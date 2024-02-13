#pragma once
#include "Commons.h"

#include "CameraScanner.h"

namespace Arena {
    class ISystem;
}

namespace freeture
{
    class CameraLucidArena_PHX016SScanner : public CameraScanner
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
}
