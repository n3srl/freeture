#pragma once
/**
* \file    CameraLucidArena.h
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief    THIS CLASS IMPLEMENTS:
            - ABSTRACT FACTORY
            - FACTORY METHOD

            for lucid cameras using Arena SDK
*
*/
#include "Commons.h"
#include "CameraScanner.h"

#include <memory>


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

        ~CameraLucidArena_Scanner() override;

        /**
         * Called to get the current reachable cameras.
         * Lucid cameras are GigEVision cameras and a specific tag into the name. "PHX016S"
         */
        virtual void UpdateCameraList() override;
    };
}
