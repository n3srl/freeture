#pragma once

/**
* \file    CameraLucidArena.h
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief   Use Aravis library to pilot Lucid phoenix cameras.
*
*/

#include "Commons.h"

#include "CameraScanner.h"
#include "ECamSdkType.h"

#ifdef LINUX
namespace freeture
{
    class CameraLucidAravis_Scanner : public CameraScanner
    {

    public:
        CameraLucidAravis_Scanner(CamSdkType);

        ~CameraLucidAravis_Scanner();

        virtual void UpdateCameraList() override;
    };
}
#endif
