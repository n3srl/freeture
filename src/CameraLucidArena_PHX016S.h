#pragma once
/**
* \file    CameraLucidArena.h
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief   Use ArenaSDK library to pilot Lucid phoenix cameras.
*
*/
#include "Commons.h"

#include <string>
#include <memory>

#include <GenICam.h>

#include "Camera.h"
#include "ECamPixFmt.h"

#define CHIPSET_MODEL   "Sony IMX273 CMOS"
#define MODEL           "Lucid PHX016S"
#define PIXEL_SIZE_H    3.45   
#define PIXEL_SIZE_W    3.45   
#define MIN_GAIN        0
#define MAX_GAIN        48
#define MIN_US_NORMAL   31
#define MAX_US_NORMAL   10000000
#define MIN_US_SHORT    0.8
#define MAX_US_SHORT    13.3
#define MIN_FPS         0.1
#define MAX_FPS         75.8
#define MAX_WIDTH       1440 
#define MAX_HEIGHT      1080
#define IMAGE_TIMEOUT   11000   // greater than maximum exposure
#define PIXEL_FORMAT    CamPixFmt::MONO12

template< typename T >
char* to_char_ptr(const T* ptr)
{
    return reinterpret_cast<char*>(const_cast<T*>(ptr));
}

namespace Arena {
    class ISystem;
    class IDevice;
    class DeviceInfo;
}

namespace freeture
{
    class CameraDescription;
    struct cameraParam;

    class CameraLucidArena_PHX016S: public Camera
    {
        private:
            std::shared_ptr<Arena::ISystem> m_ArenaSDKSystem;

            Arena::IDevice* m_ArenaDevice            = nullptr;
                  
            uint64_t        m_PreviousMissingPacketCount=0;
        public :
            CameraLucidArena_PHX016S(CameraDescription, cameraParam);

            ~CameraLucidArena_PHX016S();

            void getAvailablePixelFormats() override;

            bool getDeviceInfoBySerial(std::string, Arena::DeviceInfo&);
      
            bool acqStart() override;

            bool acqStart(bool) override;

            void acqStop() override;

            void grabCleanse() override;

            bool grabImage(std::shared_ptr<Frame> newFrame) override;

            void CopyFrame(std::shared_ptr<Frame>, const uint8_t*, size_t);

            void getExposureBounds(double &eMin, double &eMax) override;

            void getFPSBounds(double &fMin, double &fMax) override;

            void getGainBounds(double &gMin, double &gMax) override;

            bool getPixelFormat(CamPixFmt& format) override;

            bool getFrameSize(int &x, int &y, int &w, int &h) override;

            bool getFPS(double& value) override;

            std::string getModelName() override;

            double getGain() override;

            double getExposureTime() override;

            bool setExposureTime(double exp) override;

            bool setGain(double gain) override;

            bool setFPS(double fps) override;

            bool setFrameSize();

            bool setPixelFormat() override;

            //ABSTRACT FACTORY METHODS
            /// <summary>
            /// initialize SDK
            /// </summary>
            /// <returns></returns>
            bool initSDK() override;

            bool createDevice() override;

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
            /// Perform a device reset
            /// </summary>
            /// <returns></returns>
            bool reset() override;

            /// <summary>
            /// DEPRECATED USE INIT INSTEAD.
            /// 
            /// </summary>
            /// <returns></returns>
            bool grabInitialization() override;

            /// <summary>
            /// retreive main camera boundaries upon configuration of pixel format, width, height, x offset, y offset: 
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

            bool destroyDevice() override;

            /// <summary>
            /// check if configuration is allowed
            /// </summary>
            /// <param name=""></param>
            /// <returns></returns>
            bool configurationCheck(parameters&) override;

            double getMinExposureTime() override;
            double getTemperature() override;


            /// <summary>
            /// true if device is connected and running
            /// </summary>
            /// <returns></returns>
            bool isConnected() override;

            /// <summary>
            /// perform a connection to the device
            /// </summary>
            /// <returns></returns>
            bool connect() override;

            /// <summary>
           /// perform a disconnection from device
           /// </summary>
           /// <returns></returns>
            bool disconnect() override;


        private:
            bool setDefaultFrameSize();
            bool setSingleShotMode();
            bool setContinuousMode();
            bool setDayContinuous();
            bool setNightContinuous();
            bool setDayRegular();
            bool setNightRegular();
            bool getTemperature(std::string);
            void getStreamMissedPacketCount();
            bool checkSDKDevice();
            bool checkSDK();
            std::string getModel() override;
            
    };
}