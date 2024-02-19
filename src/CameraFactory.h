#pragma once
#include "Commons.h"

namespace freeture {
    class Camera;
    class CameraDescription;
    struct parameters;

    class CameraFactory
    {
    public:
        /// <summary>
        /// FACTORY METHOD
        /// </summary>
        /// <param name="sdk">SDK type</param>
        /// <param name="params">device parameters</param>
        /// <returns></returns>
        static Camera* createCamera(CameraDescription, parameters& );
   
    };
}