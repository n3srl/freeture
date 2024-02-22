#pragma once
#include "Commons.h"
#include "ECamPixFmt.h"

namespace freeture
{
    struct CameraSettings {
        //high frequency parameters, changed frequently - per frame
        double Gain = 0.0;
        double Exposure = 0.0;
        double FPS = 0.0;
        
        //low frequency parameters, changed rarely - per session or single shot
        bool   CustomSize = false;
        int    StartX = 0;
        int    StartY = 0;
        int    SizeWidth = 0;
        int    SizeHeight = 0;
        CamPixFmt PixelFormat = CamPixFmt::MONO8;
    };
}