#pragma once
/*
    THIS CLASS IMPLEMENTS: 
        - ABSTRACT FACTORY
        - FACTORY METHOD

    for lucid cameras using Arena SDK 
*/
#include "Commons.h"

#include <memory>

#include "CameraScanner.h"

namespace Arena {
    class ISystem;
}

namespace freeture
{
    class CameraLucidArena_Scanner : public CameraScanner
    {
    private:
        std::shared_ptr<Arena::ISystem> m_ArenaSDKSystem;

    public:
        CameraLucidArena_Scanner(CamSdkType);

        ~CameraLucidArena_Scanner();

        /**
         * Called to get the current reachable cameras.
         * Lucid cameras are GigEVision cameras and a specific tag into the name. "PHX016S"
         */
        virtual void UpdateCameraList() override;
    };
}
