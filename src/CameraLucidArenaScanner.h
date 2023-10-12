#pragma once

#include "CameraScanner.h"

class CameraLucidArenaScanner: public CameraScanner
{

        public:
            CameraLucidArenaScanner(CamSdkType);

            virtual void UpdateCameraList() override;

};
