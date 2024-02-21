#pragma once
/**
* \file    CameraLucidArena.h
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief   Use Aravis library to pilot Gige cameras.
*
*/




#include "Commons.h"
#include "CameraScanner.h"








namespace freeture
{
    class CameraGigeAravis_Scanner : public CameraScanner
    {

    public:
        CameraGigeAravis_Scanner(CamSdkType);

        ~CameraGigeAravis_Scanner() override;

        virtual void UpdateCameraList() override;

    };
}