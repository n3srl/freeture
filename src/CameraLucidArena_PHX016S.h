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
#define PIXEL_SIZE_H    3.45   
#define PIXEL_SIZE_W    3.45   
#define MIN_GAIN        0
#define MAX_GAIN        48
#define MIN_US_NORMAL   25
#define MAX_US_NORMAL   10000000
#define MIN_US_SHORT    0.8
#define MAX_US_SHORT    13.3
#define MIN_FPS         0.1
#define MAX_FPS         75.8
#define MAX_WIDTH       1440 
#define MAX_HEIGHT      1080
#define IMAGE_TIMEOUT   11000   // greater than maximum exposure

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
    class cameraParam;

    class CameraLucidArena_PHX016S: public Camera
    {
        private:
            std::shared_ptr<Arena::ISystem> m_ArenaSDKSystem;

            Arena::IDevice* m_ArenaDevice            = nullptr;
            bool            m_Streaming = false;             //true if streaming
            int             m_StartX             = 0;        // Crop starting X.
            int             m_StartY             = 0;        // Crop starting Y.
            int             m_Width              = 0;        // Camera region's width.
            int             m_Height             = 0;        // Camera region's height.
            double          m_FPS                = 0;        // Camera acquisition frequency.
            double          m_MinGain            = 0;        // Camera minimum gain.
            double          m_MaxGain            = 0;        // Camera maximum gain.
            double          m_MinExposure        = 0;        // Camera's minimum exposure time.
            double          m_MaxExposure        = 0;        // Camera's maximum exposure time.
            double          m_MinFPS             = 0;        // Camera's minimum frame rate.
            double          m_MaxFPS             = 0;        // Camera's maximum frame rate.
            int             m_Gain               = 0;        // Camera's gain.
            double          m_ExposureTime       = 0;        // Camera's exposure time.
            
            std::string   m_PixelFormat;                  // Image format.
           
            //uint64_t         nbCompletedBuffers = 0;        // Number of frames successfully received.
            //uint64_t         nbFailures = 0;        // Number of frames failed to be received.
            //uint64_t         nbUnderruns = 0;

            unsigned int    payload;                        // Width x height.
            //const char*     capsString          = nullptr;
            bool            shiftBitsImage;                 // For example : bits are shifted for dmk's frames.
            int             frameCounter;                   // Counter of success received frames.

        public :

            CameraLucidArena_PHX016S(CameraDescription, cameraParam);

            ~CameraLucidArena_PHX016S();


            void getAvailablePixelFormats() override;

            //getInfos

            bool getDeviceInfoBySerial(std::string, Arena::DeviceInfo&);

            //getCameraName

            //getDeviceType

            //getStopStatus

            bool acqStart() override;
            bool acqStart(bool);

            void acqStop() override;

            void grabCleanse() override;

            bool grabImage(Frame& newFrame) override;
            void CopyFrame(Frame&, char*);

            bool grabSingleImage(Frame& frame) override;

            void getExposureBounds(double &eMin, double &eMax) override;

            void getFPSBounds(double &fMin, double &fMax) override;

            void getGainBounds(double &gMin, double &gMax) override;

            bool getPixelFormat(CamPixFmt& format) override;

            bool getFrameSize(int &x, int &y, int &w, int &h) override;

            bool getFPS(double& value) override;

            //getFPSenum

            std::string getModelName() override;

            double getGain() override;

            double getExposureTime() override;

            bool setExposureTime(double exp) override;

            bool setGain(double gain) override;

            bool setFPS(double fps) override;

            bool setSize( int, int, int, int, bool) override;

            bool setPixelFormat(CamPixFmt depth) override;

            //getDataSetStatus
            //loadNextDataSet


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

            /// <summary>
            /// check if configuration is allowed
            /// </summary>
            /// <param name=""></param>
            /// <returns></returns>
            bool configurationCheck(parameters&) override;

        private:
            bool setCustomFrameSize( int, int, int, int );
            bool setDefaultFrameSize();
            bool setSingleShotMode();
            bool setContinuousMode();
            bool setFrameSize(unsigned int, unsigned int, unsigned int, unsigned int);
            bool setDayContinuous();
            bool setNightContinuous();
            bool setDayRegular();
            bool setNightRegular();
            bool getTemperature(std::string);
    };
}