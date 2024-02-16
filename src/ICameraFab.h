#pragma once
#include "Commons.h"
#include "SParam.h"

namespace freeture
{
    class ICameraFab
    {
    public:
        /// <summary>
        /// initialize SDK
        /// </summary>
        /// <returns></returns>
        virtual bool initSDK() = 0;

        /// <summary>
        /// create camera 
        /// </summary>
        /// <returns></returns>
        virtual bool createDevice() = 0;
        /**
        * Open/create a device.
        *
        * @param id Identification number of the camera to create.
        */
        virtual bool createDevice(int id) { return createDevice(); };

        virtual bool recreateDevice(int id) { return createDevice(); };

        /// <summary>
        /// init once, run configuration once (use configuration file)
        /// </summary>
        /// <returns></returns>
        virtual bool initOnce() = 0;
        /// <summary>
        /// init the camera, eg. running functions when created 
        /// CALL GRAB INITIALIZATION 
        /// </summary>
        /// <returns></returns>
        virtual bool init() = 0;

        /// <summary>
        /// DEPRECATED USE INIT INSTEAD.
        /// 
        /// </summary>
        /// <returns></returns>
        virtual bool grabInitialization() = 0;

        /// <summary>
        /// retreive main camera boundaries upon configuration: 
        ///     - fps
        ///     - gain
        ///     - exposure time
        /// </summary>
        virtual void fetchBounds(parameters&) = 0;

        /// <summary>
        /// configure the camera with the given parameters
        /// </summary>
        /// <param name=""></param>
        virtual void configure(parameters&) = 0;

        /// <summary>
        /// check if configuration is allowed
        /// </summary>
        /// <param name=""></param>
        /// <returns></returns>
        virtual bool configurationCheck(parameters&) = 0;
    };

}