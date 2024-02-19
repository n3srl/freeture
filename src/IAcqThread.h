#pragma once
#include <string>

namespace freeture {
    struct cameraParam;
    struct framesParam;
    struct videoParam;
    class Frame;

    class IAcqThread {
    public:
        /// <summary>
        /// Apply default settings values to MEMORY
        /// </summary>
        /// <param name="cp"></param>
        /// <param name="fp"></param>
        /// <param name="vp"></param>
        virtual void Setup(cameraParam cp, framesParam fp, videoParam vp) = 0;

        virtual bool runContinuousCapture(Frame& img) = 0;

        /// <summary>
        /// Called by thread in order to take 
        ///     - Regular
        ///     - Scheduled
        /// </summary>
        /// <param name="frame"></param>
        /// <returns></returns>
        virtual bool runSingleCapture(Frame& frame) = 0;

        /// <summary>
        /// Set in memory, not on device
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        virtual bool setCameraExposureTime(double value) = 0;

        /// <summary>
        /// Set in memory, not on device
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        virtual bool setCameraGain(double value) = 0;
        /// <summary>
        ///  Apply on device the in-memory value of FPS
        /// </summary>
        /// <returns></returns>
        virtual bool setCameraSize() = 0;

        /// <summary>
        /// Apply on device the in-memory value of FPS
        /// </summary>
        /// <returns></returns>
        virtual bool setCameraFPS() = 0;

        /// <summary>
        /// Apply on device the in-memory value of FPS
        /// </summary>
        /// <returns></returns>
        virtual bool setCameraPixelFormat() = 0;

  
        /// <summary>
        /// Start acquisition on device, apply memory configuration
        /// </summary>
        /// <returns></returns>
        virtual bool startCamera() = 0;

        /// <summary>
        /// Stop acquisition on device, clear structures
        /// </summary>
        /// <returns></returns>
        virtual bool stopCamera() = 0;

        // rename this for a better comprension
        virtual bool getCameraStatus() = 0;
        virtual bool getCameraDataSetStatus() = 0;
        virtual bool loadNextCameraDataSet(std::string& location) = 0;
        virtual bool getExposureStatus() = 0;
        virtual bool initializeCamera() = 0;

        // function to be deleted, bounds are ownership of the camera
        virtual void getCameraExposureBounds() = 0;
        virtual void getCameraGainBounds() = 0;
        virtual void getCameraFPSBounds() = 0;


        //function to be deleted night and day ownership is on AcqThread
//         virtual int getDayExposureTime() = 0;
//         virtual int getDayGain() = 0;
//         virtual bool setCameraDayExposureTime() = 0;
//         virtual bool setCameraDayGain() = 0;

//         virtual int getNightExposureTime() = 0;
//         virtual int getNightGain() = 0;
//         virtual bool setCameraNightExposureTime() = 0;
//         virtual bool setCameraNightGain() = 0;
    };
}