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








namespace freeture
{
    class CameraLucidAravis_Scanner : public CameraScanner
    {

    public:
        CameraLucidAravis_Scanner(CamSdkType);

        ~CameraLucidAravis_Scanner() override;

        virtual void UpdateCameraList() override;
    };
}
