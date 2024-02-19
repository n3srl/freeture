#pragma once
#include "Commons.h"
#include "SParam.h"

namespace freeture
{
    /// <summary>
    /// this class define "creation steps" for cameras. 
    /// theese methods are called in order.
    /// </summary>
    class ICameraFab
    {
    public:
        /// <summary>
        /// 1 - initialize SDK
        /// </summary>
        /// <returns></returns>
        virtual bool initSDK() = 0;

        /// <summary>
        /// 2 - create camera object from a SDK point of view.
        /// </summary>
        /// <returns></returns>
        virtual bool createDevice() = 0;

        /// <summary>
        /// 2 -  Open/create a device. IGNORE ID
        /// this method will be DEPRECATED in future releases
        /// </summary>
        /// <param name="id">IGNORED</param>
        /// <returns></returns>
        virtual bool createDevice(int id) { return createDevice(); };

        /// <summary>
        /// NOT USED
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        virtual bool recreateDevice(int id) { return createDevice(); };

        /// <summary>
        /// 3 - init once, run configuration once (use configuration file to enable this step)
        /// </summary>
        /// <returns></returns>
        virtual bool initOnce() = 0;
        /// <summary>
        /// 4 - init the camera, eg. running functions when created like set minimum default values
        /// </summary>
        /// <returns></returns>
        virtual bool init() = 0;

        /// <summary>
        /// 4 - DEPRECATED USE INIT INSTEAD. MOVE THIS FUNCTION TO DETTHREAD INTERFACE
        /// this method will be DEPRECATED in future releases
        /// </summary>
        /// <returns></returns>
        virtual bool grabInitialization() = 0;

        /// <summary>
        /// 6 - configure the camera with the given parameters
        /// </summary>
        /// <param name=""></param>
        virtual void configure(parameters&) = 0;

        /// <summary>
        /// 5 - check if configuration is allowed
        /// </summary>
        /// <param name=""></param>
        /// <returns></returns>
        virtual bool configurationCheck(parameters&) = 0;

        /// <summary>
        /// NOT USED
        /// retreive main camera boundaries upon configuration: 
        ///     - fps
        ///     - gain
        ///     - exposure time
        /// </summary>
        virtual void fetchBounds(parameters&) = 0;



        /// <summary>
        /// called when you want to destroy the camera object
        /// </summary>
        /// <returns></returns>
        virtual bool destroyDevice() = 0;

       
    };

}