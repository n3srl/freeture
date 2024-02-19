#pragma once
#include "Commons.h"
#include "ECamPixFmt.h"

namespace freeture
{
    struct CameraSettings {
        //high frequency parameters, changed frequently - per frame
        double Gain;
        double Exposure;
        double FPS;
        
        //low frequency parameters, changed rarely - per session or single shot
        bool   CustomSize;
        int    StartX;
        int    StartY;
        int    SizeWidth;
        int    SizeHeight;
        CamPixFmt PixelFormat;
    };
}