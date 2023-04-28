/**
* \file    CameraLucidArena_PHX016S.cpp
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.0
* \date    03/15/2023
* \brief   Use Arena SDK library to pilot Lucid PHX016S Cameras
*
*/
#include <iostream>

#include <time.h>
#include <algorithm>

#include "config.h"


#include "ELogSeverityLevel.h"
#include "EParser.h"

#include "Frame.h"
#include "TimeDate.h"
#include "Camera.h"
#include "ErrorManager.cpp"

#include "CameraLucidArena_PHX016S.h"

#include <ArenaApi.h>
#include <SaveApi.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#define BOOST_LOG_DYN_LINK 1
#include <boost/log/common.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/core.hpp>

using namespace freeture;
using namespace std;



    boost::log::sources::severity_logger< LogSeverityLevel >  freeture::CameraLucidArena_PHX016S::logger;

    freeture::CameraLucidArena_PHX016S::Init freeture::CameraLucidArena_PHX016S::initializer;

    /**
     * CTor.
     */
    freeture::CameraLucidArena_PHX016S::CameraLucidArena_PHX016S(bool shift):
    camera(NULL), mStartX(0), mStartY(0), mWidth(0), mHeight(0), fps(0), gainMin(0.0), gainMax(0.0),
    payload(0), exposureMin(0), exposureMax(0),fpsMin(0),fpsMax(0), gain(0), exp(0), nbCompletedBuffers(0),
    nbFailures(0), nbUnderruns(0), frameCounter(0), shiftBitsImage(shift), stream(NULL)
    {
        mExposureAvailable = true;
        mGainAvailable = true;
        mInputDeviceType = CAMERA;
    }

    /**
     * Default CTor.
     */
    freeture::CameraLucidArena_PHX016S::CameraLucidArena_PHX016S():
    camera(NULL), mStartX(0), mStartY(0), mWidth(0), mHeight(0), fps(0), gainMin(0.0), gainMax(0.0),
    payload(0), exposureMin(0), exposureMax(0), gain(0), exp(0), nbCompletedBuffers(0),
    nbFailures(0), nbUnderruns(0), frameCounter(0), shiftBitsImage(false), stream(NULL)
    {
        mExposureAvailable = true;
        mGainAvailable = true;
        mInputDeviceType = CAMERA;
    }


    /**
     *
     */
    bool freeture::CameraLucidArena_PHX016S::createDevice(int id)
    {
        std::cout << "CREO ARENASDK LUCID CAM" << std::endl;
        try
        {
            if (m_ArenaSDKSystem == nullptr)
                m_ArenaSDKSystem =  Arena::OpenSystem();
        }
        catch (std::exception& e)
        {
            ErrorManager::Exception(e);
        }

        string deviceName;

        if(!getDeviceNameById(id, deviceName))
            return false;
        std::cout << "DEVICE NAME " << deviceName << std::endl;
        vector<Arena::DeviceInfo> deviceInfos = m_ArenaSDKSystem->GetDevices();
        Arena::DeviceInfo& device_info = deviceInfos[id];
        m_Camera = m_ArenaSDKSystem->CreateDevice(device_info);
        m_Id = id;

        if(m_Camera == nullptr)
        {

            BOOST_LOG_SEV(logger, fail) << "Fail to connect the camera.";
            return false;
        }

        getFPSBounds(fpsMin,fpsMax);
        setFPS(fpsMin);

        return true;
    }

    void freeture::CameraLucidArena_PHX016S::getFPSBounds(double &fMin, double &fMax)
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return;
        }

       double fpsMin = 0.0;
       double fpsMax = 0.0;

       GenApi::CFloatPtr pAcquisitionFrameRate = m_Camera->GetNodeMap()->GetNode("AcquisitionFrameRate");

       if (!pAcquisitionFrameRate)
       {
            ErrorManager::ArenaSDKError("AcquisitionFrameRateEnable node not found");
            return;
       }

       fpsMax = pAcquisitionFrameRate->GetMax();
       fpsMin = pAcquisitionFrameRate->GetMin();

       fMin = fpsMin;
       fMax = fpsMax;
    }

    bool freeture::CameraLucidArena_PHX016S::setFPS(double fps)
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }


        bool pAcquisitionFrameRateEnable = m_Camera->GetNodeMap()->GetNode("AcquisitionFrameRateEnable");

        if (!pAcquisitionFrameRateEnable)
        {
            ErrorManager::ArenaSDKError("AcquisitionFrameRateEnable node not found");
            return false;
        }

        Arena::SetNodeValue<double>(m_Camera->GetNodeMap(), "AcquisitionFrameRate", fps);

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::getDeviceNameById(int id, string &device)
    {
        try
        {
            if (m_ArenaSDKSystem == nullptr)
                m_ArenaSDKSystem =  Arena::OpenSystem();
        }
        catch (std::exception& e)
        {
            ErrorManager::Exception(e);
        }

        m_ArenaSDKSystem->UpdateDevices(100);

        vector<Arena::DeviceInfo> deviceInfos = m_ArenaSDKSystem->GetDevices();
        int n_devices = deviceInfos.size();

        for(int i = 0; i< n_devices; i++){

            if(id == i){
                device = deviceInfos[i].ModelName();
                return true;

            }
        }


        BOOST_LOG_SEV(logger, fail) << "Fail to retrieve camera with this ID.";
        return false;
    }

    string freeture::CameraLucidArena_PHX016S::getModelName()
    {
        vector<Arena::DeviceInfo> deviceInfos = m_ArenaSDKSystem->GetDevices();
        Arena::DeviceInfo& device_info = deviceInfos[m_Id];

        return device_info.ModelName().c_str();
    }

    void freeture::CameraLucidArena_PHX016S::getExposureBounds(double &eMin, double &eMax)
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return;
        }

        double exposureMin = 0.0;
        double exposureMax = 0.0;

        GenApi::CFloatPtr pExposureTime = m_Camera->GetNodeMap()->GetNode("ExposureTime");

        if (!pExposureTime)
        {
            ErrorManager::ArenaSDKError("ExposureTime node not found");
            return;
        }

        exposureMax = pExposureTime->GetMax();
        exposureMin = pExposureTime->GetMin();

        eMin = exposureMin;
        eMax = exposureMax;
    }

    double freeture::CameraLucidArena_PHX016S::getExposureTime()
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return -1.0;
        }

    	GenApi::CFloatPtr pExposureTime = m_Camera->GetNodeMap()->GetNode("ExposureTime");

        if (!pExposureTime)
        {
            ErrorManager::ArenaSDKError("ExposureTime node not found");
            return -1;
        }

        double result = pExposureTime->GetValue();

        return result;
    }

    bool freeture::CameraLucidArena_PHX016S::setExposureTime( double val )
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }

        double expMin, expMax;

        GenApi::CFloatPtr pExposureTime = m_Camera->GetNodeMap()->GetNode("ExposureTime");

        if (! pExposureTime)
        {
            ErrorManager::ArenaSDKError("ExposureTime node not found");
            return false;
        }

        expMax = pExposureTime->GetMax();
        expMin = pExposureTime->GetMin();

        if(val >= expMin && val <= expMax)
        {
            exp = val;

            Arena::SetNodeValue<double>( m_Camera->GetNodeMap(), "ExposureTime", exp );
        }
        else
        {
            cout << "> Exposure value (" << val << ") is not in range [ " << expMin << " - " << expMax << " ]" << endl;
            return false;
        }

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::setGain(double val)
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }

        double gMin, gMax;

        GenApi::CFloatPtr pGain = m_Camera->GetNodeMap()->GetNode("Gain");

        if (!pGain )
        {
             ErrorManager::ArenaSDKError("Gain node not found");
             return false;
        }


        gMax=pGain->GetMax();
        gMin=pGain->GetMin();


        if((double)val >= gMin && (double)val <= gMax)
        {
            gain = val;
            Arena::SetNodeValue<double>( m_Camera->GetNodeMap(), "Gain", exp );
        }
        else
        {
            cout << "> Gain value (" << val << ") is not in range [ " << gMin << " - " << gMax << " ]" << endl;
            BOOST_LOG_SEV(logger, fail) << "> Gain value (" << val << ") is not in range [ " << gMin << " - " << gMax << " ]";
            return false;
        }

        return true;
    }

    double freeture::CameraLucidArena_PHX016S::getGain()
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }

        GenApi::CFloatPtr pGain = m_Camera->GetNodeMap()->GetNode("Gain");

        if (!pGain )
        {
             ErrorManager::ArenaSDKError("Gain node not found");
             return false;
        }

        return pGain->GetValue();
    }


    void freeture::CameraLucidArena_PHX016S::getGainBounds(double &gMin, double &gMax)
    {

        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return;
        }

        double gainMin = 0.0;
        double gainMax = 0.0;

        GenApi::CFloatPtr pGain = m_Camera->GetNodeMap()->GetNode("Gain");

        if (!pGain )
        {
             ErrorManager::ArenaSDKError("Gain node not found");
             return;
        }

        gainMax=pGain->GetMax();
        gainMin=pGain->GetMin();

        gMin = gainMin;
        gMax = gainMax;

    }

    bool freeture::CameraLucidArena_PHX016S::getFPS(double &value)
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }

        GenApi::CFloatPtr pAcquisitionFrameRate = m_Camera->GetNodeMap()->GetNode("AcquisitionFrameRate");

        if (!pAcquisitionFrameRate )
        {
             ErrorManager::ArenaSDKError("AcquisitionFrameRate node not found");
             return false;
        }

        value = pAcquisitionFrameRate->GetValue();

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::getFrameSize(int &x, int &y, int &w, int &h)
    {

        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }

        int64_t ww = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "Width");
        int64_t hh = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "Height");

        int64_t xx = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "OffsetX");
        int64_t yy = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "OffsetY");

        x = xx;
        y = yy;
        w = ww;
        h = hh;

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::setCustomFrameSize(int startx, int starty, int width, int height)
    {
        int64_t xx = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "OffsetX");

        if (xx>0)
        {
            cout << "Starting from : " << mStartX << "," << mStartY;
            BOOST_LOG_SEV(logger, notification) << "Starting from : " << mStartX << "," << mStartY;
        }
        else
        {
            BOOST_LOG_SEV(logger, warning) << "OffsetX, OffsetY are not available: cannot set offset.";
            return false;
        }

        Arena::SetNodeValue<int64_t>( m_Camera->GetNodeMap(), "OffsetX", startx );
        Arena::SetNodeValue<int64_t>( m_Camera->GetNodeMap(), "OffsetY", starty );
        Arena::SetNodeValue<int64_t>( m_Camera->GetNodeMap(), "Width"  , width );
        Arena::SetNodeValue<int64_t>( m_Camera->GetNodeMap(), "Height" , height );


        mStartX = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "OffsetX");
        mStartY = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "OffsetY");
        mWidth  = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "Width");
        mHeight = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "Height");


        BOOST_LOG_SEV(logger, notification) << "Camera region size : " << mWidth << "x" << mHeight;

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::setDefaultFrameSize()
    {
        int64_t sensor_width  = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "SensorWidth");
        int64_t sensor_height = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "SensorHeigth");

        BOOST_LOG_SEV(logger, notification) << "Camera sensor size : " << sensor_width << "x" << sensor_height;

        Arena::SetNodeValue<int64_t>( m_Camera->GetNodeMap(), "OffsetX", 0 );
        Arena::SetNodeValue<int64_t>( m_Camera->GetNodeMap(), "OffsetY", 0 );

        Arena::SetNodeValue<int64_t>( m_Camera->GetNodeMap(), "Width" , sensor_width );
        Arena::SetNodeValue<int64_t>( m_Camera->GetNodeMap(), "Height", sensor_height );

        mWidth  = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "Width");
        mHeight = Arena::GetNodeValue<int64_t>(m_Camera->GetNodeMap(), "Height");
    }

    bool freeture::CameraLucidArena_PHX016S::setSize(int startx, int starty, int width, int height, bool customSize)
    {

        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }

        if(customSize)
        {
            return setCustomFrameSize(startx,starty,width,height);
        }
        else
        {
            return setDefaultFrameSize();
        }

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::getPixelFormat(CamPixFmt &format)
    {

        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }

    	GenICam::gcstring _pixFormat = Arena::GetNodeValue<GenICam::gcstring>(m_Camera->GetNodeMap(), "PixelFormat");

        string pixFormat (_pixFormat.c_str());

        if (pixFormat == "Mono8")
            format = MONO8;
        else
        if (pixFormat == "Mono12")
            format = MONO12;
        else
        if (pixFormat == "Mono16")
            format = MONO16;
        else
            return false;

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::setPixelFormat(CamPixFmt depth)
    {

        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }


        switch(depth)
        {

            case MONO8 :
            {
                Arena::SetNodeValue<GenICam::gcstring>(m_Camera->GetNodeMap(), "PixelFormat", "Mono8");
            }
            break;

            case MONO12 :
            {
                Arena::SetNodeValue<GenICam::gcstring>(m_Camera->GetNodeMap(), "PixelFormat", "Mono12");
            }
            break;
            case MONO16 :
            {
                Arena::SetNodeValue<GenICam::gcstring>(m_Camera->GetNodeMap(), "PixelFormat", "Mono16");
            }
            break;
        }



        return true;
    }

    void freeture::CameraLucidArena_PHX016S::getAvailablePixelFormats()
    {
                cout << ">> Device pixel formats (firmware ver: 1.65.0 - 01/2023  :" << endl;
                cout << "- Mono8" << endl;
                cout << "- Mono10" << endl;
                cout << "- Mono10p" << endl;
                cout << "- Mono10Packed" << endl;
                cout << "- Mono12" << endl;
                cout << "- Mono12p" << endl;
                cout << "- Mono12Packed" << endl;
                cout << "- Mono16" << endl;
                cout << "- Mono24" << endl;
                cout << "- PolarizeMono8" << endl;
                cout << "- PolarizeMono12" << endl;
                cout << "- PolarizeMono12p" << endl;
                cout << "- PolarizeMono12Packed" << endl;
                cout << "- PolarizeMono16" << endl;
                cout << "- BayerGR8" << endl;
                cout << "- BayerRG8" << endl;
                cout << "- BayerGB8" << endl;
                cout << "- BayerBG8" << endl;
                cout << "- BayerGR10" << endl;
                cout << "- BayerRG10" << endl;
                cout << "- BayerGB10" << endl;
                cout << "- BayerBG10" << endl;
                cout << "- BayerGR10p" << endl;
                cout << "- BayerRG10p" << endl;
                cout << "- BayerGB10p" << endl;
                cout << "- BayerBG10p" << endl;
                cout << "- BayerGR10Packed" << endl;
                cout << "- BayerRG10Packed" << endl;
                cout << "- BayerGB10Packed" << endl;
                cout << "- BayerBG10Packed" << endl;
                cout << "- BayerGR12" << endl;
                cout << "- BayerRG12" << endl;
                cout << "- BayerGB12" << endl;
                cout << "- BayerBG12" << endl;
                cout << "- BayerGR12p" << endl;
                cout << "- BayerRG12p" << endl;
                cout << "- BayerGB12p" << endl;
                cout << "- BayerBG12p" << endl;
                cout << "- BayerGR12Packed" << endl;
                cout << "- BayerRG12Packed" << endl;
                cout << "- BayerGB12Packed" << endl;
                cout << "- BayerBG12Packed" << endl;
                cout << "- BayerGR16" << endl;
                cout << "- BayerRG16" << endl;
                cout << "- BayerGB16" << endl;
                cout << "- BayerBG16" << endl;
                cout << "- BayerGR24" << endl;
                cout << "- BayerRG24" << endl;
                cout << "- BayerGB24" << endl;
                cout << "- BayerBG24" << endl;
                cout << "- RGB8" << endl;
                cout << "- RGB12p" << endl;
                cout << "- RGB16" << endl;
                cout << "- RGB24" << endl;
                cout << "- BGR8" << endl;
                cout << "- BGR12p" << endl;
                cout << "- BGR16" << endl;
                cout << "- BGR24" << endl;
                cout << "- YCbCr8" << endl;
                cout << "- YCbCr8_CbYCr" << endl;
                cout << "- YUV422_8" << endl;
                cout << "- YUV422_8_UYVY" << endl;
                cout << "- YCbCr411_8" << endl;
                cout << "- YUV411_8_UYYVYY" << endl;
                cout << "- PolarizedAngles_0d_45d_90d_135d_Mono8" << endl;
                cout << "- PolarizedAngles_0d_45d_90d_135d_Mono12p" << endl;
                cout << "- PolarizedAngles_0d_45d_90d_135d_Mono16" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_Mono8" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_Mono12p" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_Mono16" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_S3_Mono8" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_S3_Mono12p" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_S3_Mono16" << endl;
                cout << "- PolarizedDolpAolp_Mono8" << endl;
                cout << "- PolarizedDolpAolp_Mono12p" << endl;
                cout << "- PolarizedDolp_Mono8" << endl;
                cout << "- PolarizedDolp_Mono12p" << endl;
                cout << "- PolarizedAolp_Mono8" << endl;
                cout << "- PolarizedAolp_Mono12p" << endl;
                cout << "- PolarizedAngles_0d_45d_90d_135d_BayerRG8" << endl;
                cout << "- PolarizedAngles_0d_45d_90d_135d_BayerRG12p" << endl;
                cout << "- PolarizedAngles_0d_45d_90d_135d_BayerRG16" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_BayerRG8" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_BayerRG12p" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_BayerRG16" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_S3_BayerRG8" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_S3_BayerRG12p" << endl;
                cout << "- PolarizedStokes_S0_S1_S2_S3_BayerRG16" << endl;
                cout << "- PolarizedDolpAolp_BayerRG8" << endl;
                cout << "- PolarizedDolpAolp_BayerRG12p" << endl;
                cout << "- PolarizedDolp_BayerRG8" << endl;
                cout << "- PolarizedDolp_BayerRG12p" << endl;
                cout << "- PolarizedAolp_BayerRG8" << endl;
                cout << "- PolarizedAolp_BayerRG12p" << endl;
                cout << "- Raw24" << endl;
                cout << "- Raw16" << endl;
                cout << "- YCbCr422_8_CbYCrY" << endl;
                cout << "- YCbCr422_16_CbYCrY" << endl;
                cout << "- YCbCr422_24_CbYCrY" << endl;
                cout << "- YCbCr411_8_CbYYCrYY" << endl;
                cout << "- YCbCr411_16_CbYYCrYY" << endl;
                cout << "- YCbCr411_24_CbYYCrYY" << endl;
                cout << "- PolarizedDolpAngle_Mono8" << endl;
                cout << "- PolarizedDolpAngle_Mono12p" << endl;
                cout << "- PolarizedDolpAngle_Mono16" << endl;
                cout << "- PolarizedDolpAngle_BayerRG8" << endl;
                cout << "- PolarizedDolpAngle_BayerRG12p" << endl;
                cout << "- PolarizedDolpAngle_BayerRG16" << endl;

                // Compare found pixel formats to currently formats supported by freeture

                cout << endl <<  ">> Available pixel formats :" << endl;
                cout << "- MONO8 available --> ID : Mono8 "<< endl;
                cout << "- MONO12 available --> ID : Mono12 " << endl;
                cout << "- MONO16 available --> ID : Mono16" << endl;

    }

    void freeture::CameraLucidArena_PHX016S::grabCleanse()
    {
    }

    void freeture::CameraLucidArena_PHX016S::acqStop()
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return;
        }

        //arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

        //cout << "Completed buffers = " << (unsigned long long) nbCompletedBuffers   << endl;
        //cout << "Failures          = " << (unsigned long long) nbFailures           << endl;
        //cout << "Underruns         = " << (unsigned long long) nbUnderruns          << endl;

        BOOST_LOG_SEV(logger, notification) << "Completed buffers = " << (unsigned long long) nbCompletedBuffers;
        BOOST_LOG_SEV(logger, notification) << "Failures          = " << (unsigned long long) nbFailures;
        BOOST_LOG_SEV(logger, notification) << "Underruns         = " << (unsigned long long) nbUnderruns;

        BOOST_LOG_SEV(logger, notification) << "Stopping acquisition...";

      	m_Camera->StopStream();

        BOOST_LOG_SEV(logger, notification) << "Acquisition stopped.";

        BOOST_LOG_SEV(logger, notification) << "Unreferencing stream.";
        //g_object_unref(stream);
        //stream = NULL;

        BOOST_LOG_SEV(logger, notification) << "Unreferencing camera.";
        m_ArenaSDKSystem->DestroyDevice(m_Camera);
        m_Camera = nullptr;
    }

    bool freeture::CameraLucidArena_PHX016S::acqStart()
    {
        if (m_Camera == nullptr)
        {
            ErrorManager::ArenaSDKError("Camera is nullptr");
            return false;
        }

    	// Set acquisition mode
        //    Set acquisition mode before starting the stream. Starting the stream
        //    requires the acquisition mode to be set beforehand. The acquisition
        //    mode controls the number of images a device acquires once the stream
        //    has been started. Setting the acquisition mode to 'Continuous' keeps
        //    the stream from stopping. This example returns the camera to its
        //    initial acquisition mode near the end of the example.
        BOOST_LOG_SEV(logger, notification) << "Set camera to CONTINUOUS MODE";
      	Arena::SetNodeValue<GenICam::gcstring>( m_Camera->GetNodeMap(), "AcquisitionMode", "Continuous");

      	// Set buffer handling mode
        //    Set buffer handling mode before starting the stream. Starting the
        //    stream requires the buffer handling mode to be set beforehand. The
        //    buffer handling mode determines the order and behavior of buffers in
        //    the underlying stream engine. Setting the buffer handling mode to
        //    'NewestOnly' ensures the most recent image is delivered, even if it
        //    means skipping frames.
      	BOOST_LOG_SEV(logger, notification) << "Set buffer handling mode to 'NewestOnly'";
        Arena::SetNodeValue<GenICam::gcstring>( m_Camera->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");

        // Enable stream auto negotiate packet size
		//    Setting the stream packet size is done before starting the stream.
		//    Setting the stream to automatically negotiate packet size instructs
		//    the camera to receive the largest packet size that the system will
		//    allow. This generally increases frame rate and results in fewer
		//    interrupts per image, thereby reducing CPU load on the host system.
		//    Ethernet settings may also be manually changed to allow for a
		//    larger packet size.
        BOOST_LOG_SEV(logger, notification) << "Enable stream to auto negotiate packet size";

        // Disable stream packet resend
		//    Enable stream packet resend before starting the stream. Images are
		//    sent from the camera to the host in packets using UDP protocol,
		//    which includes a header image number, packet number, and timestamp
		//    information. If a packet is missed while receiving an image, a
		//    packet resend is requested and this information is used to retrieve
		//    and redeliver the missing packet in the correct order.
        BOOST_LOG_SEV(logger, notification) << "Disable stream packet resend";
		Arena::SetNodeValue<bool>( m_Camera->GetTLStreamNodeMap(), "StreamPacketResendEnable", false);


		Arena::SetNodeValue<bool>( m_Camera->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

        BOOST_LOG_SEV(logger, notification) << "Set camera TriggerMode to Off";
        Arena::SetNodeValue<GenICam::gcstring>( m_Camera->GetNodeMap(), "TriggerMode", "Off");

        // Start stream
        //    Start the stream before grabbing any images. Starting the stream
        //    allocates buffers, which can be passed in as an argument (default: 10),
        //    and begins filling them with data. Starting the stream blocks write
        //    access to many features such as width, height, and pixel format, as
        //    well as acquisition and buffer handling modes, among others. The stream
        //    needs to be stopped later.
        BOOST_LOG_SEV(logger, notification) << "Start acquisition on camera";
        m_Camera->StartStream();

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::grabInitialization()
    {
        frameCounter = 0;

        BOOST_LOG_SEV(logger, notification) << "Camera payload (NOT USED): " << payload;

        pixFormat == Arena::GetNodeValue<GenICam::gcstring>(m_Camera->GetNodeMap(), "PixelFormat");

        getFPSBounds(fpsMin,fpsMax);

        BOOST_LOG_SEV(logger, notification) << "Camera FPS bound min : " << fpsMin;
        BOOST_LOG_SEV(logger, notification) << "Camera FPS bound max : " << fpsMax;

        getExposureBounds(exposureMin,exposureMax);

        BOOST_LOG_SEV(logger, notification) << "Camera exposure bound min : " << exposureMin;
        BOOST_LOG_SEV(logger, notification) << "Camera exposure bound max : " << exposureMax;

        getGainBounds(gainMin,gainMax);

        BOOST_LOG_SEV(logger, notification) << "Camera gain bound min : " << gainMin;
        BOOST_LOG_SEV(logger, notification) << "Camera gain bound max : " << gainMax;

        BOOST_LOG_SEV(logger, notification) << "Camera frame rate : " << fps;

        capsString = pixFormat.c_str();

        BOOST_LOG_SEV(logger, notification) << "Camera format : " << capsString;

        gain = getGain();

        BOOST_LOG_SEV(logger, notification) << "Camera gain : " << gain;

        exp = getExposureTime();

        BOOST_LOG_SEV(logger, notification) << "Camera exposure : " << exp;

        cout << endl;

        cout << "DEVICE SELECTED : " << m_Id    << endl;

        string model_name;

        getDeviceNameById(m_Id, model_name);

        cout << "DEVICE NAME     : " << model_name  << endl;

        cout << "DEVICE VENDOR   : " << "Lucid" << endl;

        cout << "PAYLOAD         : " << payload                             << endl;
        cout << "Start X         : " << mStartX                             << endl
             << "Start Y         : " << mStartY                             << endl;
        cout << "Width           : " << mWidth                               << endl
             << "Height          : " << mHeight                              << endl;
        cout << "Exp Range       : [" << exposureMin    << " - " << exposureMax   << "]"  << endl;
        cout << "Exp             : " << exp                                 << endl;
        cout << "Gain Range      : [" << gainMin        << " - " << gainMax       << "]"  << endl;
        cout << "Gain            : " << gain                                << endl;
        cout << "Fps             : " << fps                                 << endl;
        cout << "Type            : " << capsString                         << endl;

        cout << endl;


      	// Set buffer handling mode
        //    Set buffer handling mode before starting the stream. Starting the
        //    stream requires the buffer handling mode to be set beforehand. The
        //    buffer handling mode determines the order and behavior of buffers in
        //    the underlying stream engine. Setting the buffer handling mode to
        //    'NewestOnly' ensures the most recent image is delivered, even if it
        //    means skipping frames.
      	BOOST_LOG_SEV(logger, notification) << "Set buffer handling mode to 'NewestOnly'";
        Arena::SetNodeValue<GenICam::gcstring>( m_Camera->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");

        // Enable stream auto negotiate packet size
		//    Setting the stream packet size is done before starting the stream.
		//    Setting the stream to automatically negotiate packet size instructs
		//    the camera to receive the largest packet size that the system will
		//    allow. This generally increases frame rate and results in fewer
		//    interrupts per image, thereby reducing CPU load on the host system.
		//    Ethernet settings may also be manually changed to allow for a
		//    larger packet size.
        BOOST_LOG_SEV(logger, notification) << "Enable stream to auto negotiate packet size";
		Arena::SetNodeValue<bool>( m_Camera->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

        // Disable stream packet resend
		//    Enable stream packet resend before starting the stream. Images are
		//    sent from the camera to the host in packets using UDP protocol,
		//    which includes a header image number, packet number, and timestamp
		//    information. If a packet is missed while receiving an image, a
		//    packet resend is requested and this information is used to retrieve
		//    and redeliver the missing packet in the correct order.
        BOOST_LOG_SEV(logger, notification) << "Disable stream packet resend";
		Arena::SetNodeValue<bool>( m_Camera->GetTLStreamNodeMap(), "StreamPacketResendEnable", false);

        // Create a new stream object. Open stream on Camera.
        m_Camera->StartStream();

        Arena::IImage* pImage = m_Camera->GetImage(IMAGE_TIMEOUT);

        m_Camera->RequeueBuffer(pImage);

        return true;
    }

    bool freeture::CameraLucidArena_PHX016S::grabSingleImage(Frame &frame, int camID)
    {
        bool res = false;

        if(!createDevice(camID))
            return false;

        if(!setPixelFormat(frame.mFormat))
            return false;

        if(!setExposureTime(frame.mExposure))
            return false;

        if(!setGain(frame.mGain))
            return false;

        if(frame.mWidth > 0 && frame.mHeight > 0)
        {
            setCustomFrameSize(frame.mStartX, frame.mStartY, frame.mWidth, frame.mHeight);
        }
        else
        {
            setDefaultFrameSize();
        }

   		Arena::IImage* pImage = m_Camera->GetImage(IMAGE_TIMEOUT);

        size_t size = pImage->GetSizeFilled();
		size_t width = pImage->GetWidth();
		size_t height = pImage->GetHeight();

		pixFormat = GetPixelFormatName(static_cast<PfncFormat>(pImage->GetPixelFormat()));

		uint64_t timestampNs = pImage->GetTimestampNs();

		std::cout << " (" << size << " bytes; " << width << "x" << height << "; " << pixFormat << "; timestamp (ns): " << timestampNs << ")";

		// Requeue image buffer
		//    Requeue an image buffer when access to it is no longer needed.
		//    Notice that failing to requeue buffers may cause memory to leak and
		//    may also result in the stream engine being starved due to there
		//    being no available buffers.
		std::cout << " and requeue\n";

		m_Camera->RequeueBuffer(pImage);


        getExposureBounds(exposureMin,exposureMax);
        getGainBounds(gainMin, gainMax);

        setFPS( frame.mFps); /* Regular captures */

        getFPS(fps);

        capsString =  pixFormat.c_str();

        gain = getGain();

        exp     = getExposureTime();

        cout << endl;

        cout << "DEVICE SELECTED : " << m_Id    << endl;

        string model_name;

        getDeviceNameById(m_Id, model_name);


        cout << "DEVICE NAME     : " << model_name   << endl;

        cout << "DEVICE VENDOR   : " << "LUCID" << endl;

        cout << "PAYLOAD         : " << payload                             << endl;
        cout << "Start X         : " << mStartX                             << endl
             << "Start Y         : " << mStartY                             << endl;
        cout << "Width           : " << mWidth                               << endl
             << "Height          : " << mHeight                              << endl;
        cout << "Exp Range       : [" << exposureMin    << " - " << exposureMax   << "]"  << endl;
        cout << "Exp             : " << exp                                 << endl;
        cout << "Gain Range      : [" << gainMin        << " - " << gainMax       << "]"  << endl;
        cout << "Gain            : " << gain                                << endl;
        cout << "Fps             : " << fps                                 << endl;
        cout << "Type            : " << capsString                         << endl;

        cout << endl;

      	// Set buffer handling mode
        //    Set buffer handling mode before starting the stream. Starting the
        //    stream requires the buffer handling mode to be set beforehand. The
        //    buffer handling mode determines the order and behavior of buffers in
        //    the underlying stream engine. Setting the buffer handling mode to
        //    'NewestOnly' ensures the most recent image is delivered, even if it
        //    means skipping frames.
      	BOOST_LOG_SEV(logger, notification) << "Set buffer handling mode to 'NewestOnly'";
        Arena::SetNodeValue<GenICam::gcstring>( m_Camera->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");

        // Enable stream auto negotiate packet size
		//    Setting the stream packet size is done before starting the stream.
		//    Setting the stream to automatically negotiate packet size instructs
		//    the camera to receive the largest packet size that the system will
		//    allow. This generally increases frame rate and results in fewer
		//    interrupts per image, thereby reducing CPU load on the host system.
		//    Ethernet settings may also be manually changed to allow for a
		//    larger packet size.
        BOOST_LOG_SEV(logger, notification) << "Enable stream to auto negotiate packet size";
		Arena::SetNodeValue<bool>( m_Camera->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

        // Disable stream packet resend
		//    Enable stream packet resend before starting the stream. Images are
		//    sent from the camera to the host in packets using UDP protocol,
		//    which includes a header image number, packet number, and timestamp
		//    information. If a packet is missed while receiving an image, a
		//    packet resend is requested and this information is used to retrieve
		//    and redeliver the missing packet in the correct order.
        BOOST_LOG_SEV(logger, notification) << "Disable stream packet resend";
		Arena::SetNodeValue<bool>( m_Camera->GetTLStreamNodeMap(), "StreamPacketResendEnable", false);

		m_Camera->StartStream();

        // Create a new stream object. Open stream on Camera.
        stream = arv_camera_create_stream(camera, NULL, NULL, &error);
        ErrorManager::CheckAravisError(&error);

        if(stream != NULL)
        {

            // Push 50 buffer in the stream input buffer queue.
            arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));

            // Set acquisition mode to continuous.
            arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_SINGLE_FRAME, &error);
            ErrorManager::CheckAravisError(&error);

            // Very usefull to avoid arv buffer timeout status
            sleep(1);

            // Start acquisition.
            arv_camera_start_acquisition(camera, &error);
            ErrorManager::CheckAravisError(&error);

            // Get image buffer.
            ArvBuffer *arv_buffer = arv_stream_timeout_pop_buffer(stream, frame.mExposure + 5000000); //us

            char *buffer_data;
            size_t buffer_size;

            cout << ">> Acquisition in progress... (Please wait)" << endl;

            if (arv_buffer != NULL){

                if(arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS){

                    buffer_data = (char *) arv_buffer_get_data (arv_buffer, &buffer_size);

                    //Timestamping.
                    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();

                    if(pixFormat == "Mono8"){

                        Mat image = Mat(mHeight, mWidth, CV_8UC1, buffer_data);
                        image.copyTo(frame.mImg);

                    }else if(pixFormat == "Mono12" || pixFormat == "Mono16") {

                        // Unsigned short image.
                        Mat image = Mat(mHeight, mWidth, CV_16UC1, buffer_data);

                        // http://www.theimagingsource.com/en_US/support/documentation/icimagingcontrol-class/PixelformatY16.htm
                        // Some sensors only support 10-bit or 12-bit pixel data. In this case, the least significant bits are don't-care values.
                        if(shiftBitsImage && pixFormat != "Mono16" )
                        {
                            unsigned short * p;
                            for(int i = 0; i < image.rows; i++){
                                p = image.ptr<unsigned short>(i);
                                for(int j = 0; j < image.cols; j++) p[j] = p[j] >> 4;
                            }
                        }

                        image.copyTo(frame.mImg);
                    }

                    frame.mDate = TimeDate::splitIsoExtendedDate(to_iso_extended_string(time));
                    frame.mFps = arv_camera_get_frame_rate(camera, &error);
                    ErrorManager::CheckAravisError(&error);

                    res = true;

                }else{

                    switch(arv_buffer_get_status(arv_buffer)){

                        case 0 :

                            cout << "ARV_BUFFER_STATUS_SUCCESS : the buffer contains a valid image"<<endl;

                            break;

                        case 1 :

                            cout << "ARV_BUFFER_STATUS_CLEARED: the buffer is cleared"<<endl;

                            break;

                        case 2 :

                            cout << "ARV_BUFFER_STATUS_TIMEOUT: timeout was reached before all packets are received"<<endl;

                            break;

                        case 3 :

                            cout << "ARV_BUFFER_STATUS_MISSING_PACKETS: stream has missing packets"<<endl;

                            break;

                        case 4 :

                            cout << "ARV_BUFFER_STATUS_WRONG_PACKET_ID: stream has packet with wrong id"<<endl;

                            break;

                        case 5 :

                            cout << "ARV_BUFFER_STATUS_SIZE_MISMATCH: the received image didn't fit in the buffer data space"<<endl;

                            break;

                        case 6 :

                            cout << "ARV_BUFFER_STATUS_FILLING: the image is currently being filled"<<endl;

                            break;

                        case 7 :

                            cout << "ARV_BUFFER_STATUS_ABORTED: the filling was aborted before completion"<<endl;

                            break;


                    }

                    res = false;

                }

                arv_stream_push_buffer(stream, arv_buffer);

           }else{

                BOOST_LOG_SEV(logger, fail) << "Fail to pop buffer from stream.";
                res = false;

           }

            arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

            cout << ">> Completed buffers = " << (unsigned long long) nbCompletedBuffers    << endl;
            cout << ">> Failures          = " << (unsigned long long) nbFailures           << endl;
            //cout << ">> Underruns         = " << (unsigned long long) nbUnderruns          << endl;

            // Stop acquisition.
            arv_camera_stop_acquisition(camera, &error);
            ErrorManager::CheckAravisError(&error);


            g_object_unref(stream);
            stream = NULL;
            g_object_unref(camera);
            camera = NULL;

        }

        return res;

    }

    //
    // TO BE PORTED TO ARENA SDK
    //
    //
    //

    /**
     * DTor.
     */
    freeture::CameraLucidArena_PHX016S::~CameraLucidArena_PHX016S()
    {

        if(stream != nullptr)
            g_object_unref(stream);

        m_ArenaSDKSystem->DestroyDevice(m_Camera);
		Arena::CloseSystem(m_ArenaSDKSystem);
    }

    bool freeture::CameraLucidArena_PHX016S::grabImage(Frame &newFrame)
    {

        ArvBuffer *arv_buffer;
        //exp = arv_camera_get_exposure_time(camera);

        arv_buffer = arv_stream_timeout_pop_buffer(stream,2000000); //us

        char *buffer_data;
        size_t buffer_size;

        if(arv_buffer == NULL){

            throw runtime_error("arv_buffer is NULL");
            return false;

        }
        else
        {

            try{

                if ( arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS )
                {

                    //BOOST_LOG_SEV(logger, normal) << "Success to grab a frame.";

                    buffer_data = (char *) arv_buffer_get_data (arv_buffer, &buffer_size);

                    //Timestamping.
                    //string acquisitionDate = TimeDate::localDateTime(microsec_clock::universal_time(),"%Y:%m:%d:%H:%M:%S");
                    //BOOST_LOG_SEV(logger, normal) << "Date : " << acquisitionDate;
                    boost::posix_time::ptime time =  boost::posix_time::microsec_clock::universal_time();
                    string acquisitionDate = to_iso_extended_string(time);
                    //BOOST_LOG_SEV(logger, normal) << "Date : " << acqDateInMicrosec;

                    Mat image;
                    CamPixFmt imgDepth = MONO8;
                    int saturateVal = 0;

                    if(pixFormat == "Mono8")
                    {
                        //BOOST_LOG_SEV(logger, normal) << "Creating Mat 8 bits ...";
                        image = Mat(mHeight, mWidth, CV_8UC1, buffer_data);
                        imgDepth = MONO8;
                        saturateVal = 255;

                    }
                    else if(pixFormat == "Mono12")
                    {

                        //BOOST_LOG_SEV(logger, normal) << "Creating Mat 16 bits ...";
                        image = Mat(mHeight, mWidth, CV_16UC1, buffer_data);
                        imgDepth = MONO12;
                        saturateVal = 4095;

                        //double t3 = (double)getTickCount();

                        if(shiftBitsImage){

                            //BOOST_LOG_SEV(logger, normal) << "Shifting bits ...";


                                unsigned short * p;

                                for(int i = 0; i < image.rows; i++){
                                    p = image.ptr<unsigned short>(i);
                                    for(int j = 0; j < image.cols; j++)
                                        p[j] = p[j] >> 4;
                                }

                            //BOOST_LOG_SEV(logger, normal) << "Bits shifted.";

                        }

                        //t3 = (((double)getTickCount() - t3)/getTickFrequency())*1000;
                        //cout << "> Time shift : " << t3 << endl;
                    }
                    else if(pixFormat == "Mono16")
                    {
                        //BOOST_LOG_SEV(logger, normal) << "Creating Mat 16 bits ...";
                        image = Mat(mHeight, mWidth, CV_16UC1, buffer_data);
                        imgDepth = MONO16;
                        saturateVal = 65535;

                        //double t3 = (double)getTickCount();
                        //t3 = (((double)getTickCount() - t3)/getTickFrequency())*1000;
                        //cout << "> Time shift : " << t3 << endl;
                    }

                    //BOOST_LOG_SEV(logger, normal) << "Creating frame object ...";
                    newFrame = Frame(image, gain, exp, acquisitionDate);
                    //BOOST_LOG_SEV(logger, normal) << "Setting date of frame ...";
                    //newFrame.setAcqDateMicro(acqDateInMicrosec);
                    //BOOST_LOG_SEV(logger, normal) << "Setting fps of frame ...";
                    newFrame.mFps = fps;
                    newFrame.mFormat = imgDepth;
                    //BOOST_LOG_SEV(logger, normal) << "Setting saturated value of frame ...";
                    newFrame.mSaturatedValue = saturateVal;
                    newFrame.mFrameNumber = frameCounter;
                    frameCounter++;

                    //BOOST_LOG_SEV(logger, normal) << "Re-pushing arv buffer in stream ...";
                    arv_stream_push_buffer(stream, arv_buffer);

                    return true;

                }else{

                    switch(arv_buffer_get_status(arv_buffer)){

                        case 0 :
                            cout << "ARV_BUFFER_STATUS_SUCCESS : the buffer contains a valid image"<<endl;
                            break;
                        case 1 :
                            cout << "ARV_BUFFER_STATUS_CLEARED: the buffer is cleared"<<endl;
                            break;
                        case 2 :
                            cout << "ARV_BUFFER_STATUS_TIMEOUT: timeout was reached before all packets are received"<<endl;
                            break;
                        case 3 :
                            cout << "ARV_BUFFER_STATUS_MISSING_PACKETS: stream has missing packets"<<endl;
                            break;
                        case 4 :
                            cout << "ARV_BUFFER_STATUS_WRONG_PACKET_ID: stream has packet with wrong id"<<endl;
                            break;
                        case 5 :
                            cout << "ARV_BUFFER_STATUS_SIZE_MISMATCH: the received image didn't fit in the buffer data space"<<endl;
                            break;
                        case 6 :
                            cout << "ARV_BUFFER_STATUS_FILLING: the image is currently being filled"<<endl;
                            break;
                        case 7 :
                            cout << "ARV_BUFFER_STATUS_ABORTED: the filling was aborted before completion"<<endl;
                            break;

                    }

                    arv_stream_push_buffer(stream, arv_buffer);

                    return false;
                }

            }catch(exception& e){

                cout << e.what() << endl;
                BOOST_LOG_SEV(logger, critical) << e.what() ;
                return false;

            }catch(exception& e){

                cout << e.what() << endl;
                BOOST_LOG_SEV(logger, critical) << e.what() ;
                return false;

            }
        }
    }

