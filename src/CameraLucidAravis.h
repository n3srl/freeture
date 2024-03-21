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

#include <string>
#include <memory>

#include "Camera.h"
#include "ECamPixFmt.h"

#ifdef LINUX
#include "arv.h"
#include "arvinterface.h"

namespace freeture
{
    class Frame;
    class CameraDescription;
    struct cameraParam;

    class CameraLucidAravis : public Camera
    {

    private:
        GError*         error = nullptr;        // ARAVIS API Error
        ArvCamera*      camera;                // Camera to control.
        ArvPixelFormat  pixFormat;              // Image format.
        ArvStream*      stream;                // Object for video stream reception.
       
        unsigned int    payload;                // Width x height.
        const char*     capsString;
        bool            shiftBitsImage;         // For example : bits are shifted for dmk's frames.
        guint64         nbCompletedBuffers;     // Number of frames successfully received.
        guint64         nbFailures;             // Number of frames failed to be received.
        guint64         nbUnderruns;
        int             frameCounter;           // Counter of success received frames.

    public:
        CameraLucidAravis(CameraDescription, cameraParam );
        ~CameraLucidAravis();

        bool recreateDevice(int);

        void grabCleanse();

        bool acqStart();

        void acqStop();

        bool grabImage(std::shared_ptr<Frame> newFrame);

        bool grabSingleImage(std::shared_ptr<Frame> frame);

        bool getDeviceNameById(int id, std::string& device);

        void getExposureBounds(double& eMin, double& eMax);

        void getFPSBounds(double& fMin, double& fMax);

        void getGainBounds(double& gMin, double& gMax);

        bool getPixelFormat(CamPixFmt& format);

        bool getFrameSize(int& x, int& y, int& w, int& h);

        bool getFPS(double& value);

        std::string getModelName();

        double getExposureTime();

        bool setAutoExposure(bool val);

        bool setExposureTime(double exp);

        bool setGain(double gain);

        bool setFPS(double fps);

        bool setFrameSize(int startx, int starty, int width, int height, bool customSize);

        bool setPixelFormat(CamPixFmt depth);

        void saveGenicamXml(std::string p);

        bool setSize(int startx, int starty, int width, int height, bool customSize);

        void getAvailablePixelFormats();


        bool createDevice() override;

        bool destroyDevice() override;

        //ABSTRACT FACTORY METHODS
        /// <summary>
        /// initialize SDK
        /// </summary>
        /// <returns></returns>
        bool initSDK() override;

        /// <summary>
        /// init once, run configuration once (use configuration file)
        /// </summary>
        /// <returns></returns>
        bool initOnce() override;

        /// <summary>
        /// init the camera, eg. running functions when created 
        /// CALL GRAB INITIALIZATION 
        /// </summary>
        /// <returns></returns>
        bool init() override;

        /// <summary>
        /// DEPRECATED USE INIT INSTEAD.
        /// 
        /// </summary>
        /// <returns></returns>
        bool grabInitialization() override;

        /// <summary>
        /// retreive main camera boundaries upon configuration: 
        ///     - fps
        ///     - gain
        ///     - exposure time
        /// </summary>
        void fetchBounds(parameters&) override;

        /// <summary>
        /// configure the camera with the given parameters
        /// </summary>
        /// <param name=""></param>
        void configure(parameters&) override;

        /// <summary>
        /// check if configuration is allowed
        /// </summary>
        /// <param name=""></param>
        /// <returns></returns>
        bool configurationCheck(parameters&) override;

        double getMinExposureTime() override;

    };
}
#endif
