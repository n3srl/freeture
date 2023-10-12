/**
* \file    CameraLucidArena.h
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief   Use Aravis library to pilot Lucid phoenix cameras.
*
*/

#pragma once

#ifdef LINUX
#include <string>

#include <ArenaApi.h>
#include "Camera.h"

#define IMAGE_TIMEOUT 40000


namespace freeture
{

    class CameraLucidArena_PHX016S: public Camera
    {

        private:

            static boost::log::sources::severity_logger< LogSeverityLevel > logger;

            static class Init{

                public:

                    Init(){

                        logger.add_attribute("ClassName", boost::log::attributes::constant<std::string>("CameraLucidArena_PHX016S"));

                    }

            }initializer;

            Arena::ISystem* m_ArenaSDKSystem    = nullptr;
            Arena::IDevice* m_Camera            = nullptr;
            int m_Id                            = -1;

            int             mStartX             = 0;        // Crop starting X.
            int             mStartY             = 0;        // Crop starting Y.
            int             mWidth              = 0;        // Camera region's width.
            int             mHeight             = 0;        // Camera region's height.
            double          fps                 = 0;        // Camera acquisition frequency.
            double          gainMin             = 0;        // Camera minimum gain.
            double          gainMax             = 0;        // Camera maximum gain.
            double          exposureMin         = 0;        // Camera's minimum exposure time.
            double          exposureMax         = 0;        // Camera's maximum exposure time.
            double          fpsMin              = 0;        // Camera's minimum frame rate.
            double          fpsMax              = 0;        // Camera's maximum frame rate.
            int             gain                = 0;        // Camera's gain.
            double          exp                 = 0;        // Camera's exposure time.

            guint64         nbCompletedBuffers  = 0;        // Number of frames successfully received.
            guint64         nbFailures          = 0;        // Number of frames failed to be received.
            guint64         nbUnderruns         = 0;

            GenICam::gcstring   pixFormat;                  // Image format.



            GError*         error               = nullptr;  // ARAVIS API Error
            ArvCamera*      camera              = nullptr;  // Camera to control.
            ArvStream*      stream              = nullptr;  // Object for video stream reception.

            unsigned int    payload;                        // Width x height.
            const char*     capsString          = nullptr;
            bool            shiftBitsImage;                 // For example : bits are shifted for dmk's frames.
            int             frameCounter;                   // Counter of success received frames.

        public :

            CameraLucidArena_PHX016S(bool shift);

            CameraLucidArena_PHX016S();

            ~CameraLucidArena_PHX016S();


            void getAvailablePixelFormats() override;

            //getInfos

            bool createDevice(int id) override;

            bool getDeviceNameById(int id, std::string &device) override;

            //getCameraName

            //getDeviceType

            //getStopStatus

            bool grabInitialization() override;

            bool acqStart() override;

            void acqStop() override;

            void grabCleanse() override;

            bool grabImage(Frame& newFrame) override;

            bool grabSingleImage(Frame &frame, int camID) override;

            void getExposureBounds(double &eMin, double &eMax) override;

            void getFPSBounds(double &fMin, double &fMax) override;

            void getGainBounds(double &gMin, double &gMax) override;

            bool getPixelFormat(CamPixFmt &format) override;

            bool getFrameSize(int &x, int &y, int &w, int &h) override;

            bool getFPS(double &value) override;

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

            //test
        private:
            bool setCustomFrameSize( int, int, int, int );
            bool setDefaultFrameSize();


    };
}
#endif
