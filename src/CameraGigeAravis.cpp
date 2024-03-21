/*
                        CameraGigeAravis.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2016 Yoan Audureau
*                       2018 Chiara Marmo
*                               GEOPS-UPSUD-CNRS
*
*   License:        GNU General Public License
*
*   FreeTure is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*   FreeTure is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*   You should have received a copy of the GNU General Public License
*   along with FreeTure. If not, see <http://www.gnu.org/licenses/>.
*
*   Last modified:      30/03/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    CameraGigeAravis.cpp
* \author  Yoan Audureau -- Chiara Marmo -- GEOPS-UPSUD
* \version 1.2
* \date    30/03/2018
* \brief   Use Aravis library to pilot GigE Cameras.
*          https://wiki.gnome.org/action/show/Projects/Aravis?action=show&redirect=Aravis
*/

#include "CameraGigeAravis.h"

#include "Logger.h"

#include <iostream>
#include <fstream>
#include <boost/date_time/posix_time/time_formatters.hpp>

#include "Frame.h"
#include "EParser.h"
#include "CameraFirstInit.h"

#ifdef LINUX
#ifdef USE_ARAVIS 

using namespace freeture;
using namespace std;

CameraGigeAravis::CameraGigeAravis(CameraDescription description, cameraParam settings):
    Camera(description, settings),
    payload(0), nbCompletedBuffers(0),
    nbFailures(0), nbUnderruns(0), frameCounter(0), shiftBitsImage(settings.SHIFT_BITS), stream(nullptr)
{
    m_ExposureAvailable = true;
    m_GainAvailable = true;
}

    CameraGigeAravis::~CameraGigeAravis()
    {
        LOG_DEBUG << "CameraGigeAravis::~CameraGigeAravis" << endl;
        if(stream != nullptr)
            g_object_unref(stream);

        if(camera != nullptr)
            g_object_unref(camera);

    }

    bool CameraGigeAravis::createDevice()
    {
        LOG_DEBUG << "CameraGigeAravis::createDevice" << endl;

        int id = m_CameraDescriptor.DeviceId;

        string deviceName;
        if(!getDeviceNameById(id, deviceName))
            return false;

        camera = arv_camera_new(deviceName.c_str(),&error);

        if(camera == nullptr)
        {

            LOG_ERROR << "CameraGigeAravis::createDevice" << "Fail to connect the camera." << endl;
            return false;

        }

        delete error;

        return true;
    }

    bool CameraGigeAravis::setSize(int startx, int starty, int width, int height, bool customSize)
    {
        int sensor_width, sensor_height;
        arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);
        if(customSize) {

            if(width > sensor_width) 
            {
                LOG_DEBUG << width << " Bigger than the sensor width => width set to " << sensor_width << endl;
                width = sensor_width;
            }
            if(height > sensor_height)
            {
                LOG_DEBUG << height << " Bigger than the sensor height => height set to " << sensor_height << endl;
                height = sensor_height;
            }

            arv_camera_set_region(camera, startx, starty, width, height,&error);

            arv_camera_get_region (camera, &m_StartX, &m_StartY, &m_Width, &m_Height,&error);

            LOG_DEBUG << "Camera region size : " << m_Width << "x" << m_Height << endl;
            if (arv_device_get_feature(arv_camera_get_device(camera), "OffsetX")) {
                LOG_DEBUG << "Starting from : " << m_StartX << "," << m_StartY << endl;
            } else {
                LOG_WARNING << "OffsetX, OffsetY are not available: cannot set offset." << endl;
            }


        // Default is maximum size
        } else {
            LOG_DEBUG << "Camera sensor size : " << sensor_width << "x" << sensor_height << endl;

            arv_camera_set_region(camera, 0, 0, sensor_width, sensor_height, &error);
            arv_camera_get_region (camera, nullptr, nullptr, &m_Width, &m_Height, &error);
           
        }

        return true;

    }

    bool CameraGigeAravis::getDeviceNameById(int id, string &device){

        arv_update_device_list();

        int n_devices = arv_get_n_devices();

        for(int i = 0; i< n_devices; i++){

            if(id == i){

                device = arv_get_device_id(i);
                return true;

            }
        }

        LOG_ERROR << "Fail to retrieve camera with this ID." << endl;
        return false;
    }



    bool CameraGigeAravis::grabInitialization()
    {
        LOG_DEBUG <<"CameraGigeAravis::grabInitialization;" << "Camera payload : " << payload << endl;

        frameCounter = 0;

        payload = arv_camera_get_payload (camera, &error);

        LOG_DEBUG << "CameraGigeAravis::grabInitialization;" << "Camera payload : " << payload << endl;

        pixFormat = arv_camera_get_pixel_format(camera, &error);

        arv_camera_get_exposure_time_bounds (camera, &m_MinExposure, &m_MaxExposure, &error);
        
        LOG_DEBUG << "CameraGigeAravis::grabInitialization;" << "Camera exposure bound min : " << m_MinExposure << endl;
        LOG_DEBUG << "CameraGigeAravis::grabInitialization;" << "Camera exposure bound max : " << m_MaxExposure << endl;

        arv_camera_get_gain_bounds (camera, &m_MinGain, &m_MaxGain, &error);
        LOG_DEBUG << "CameraGigeAravis::grabInitialization;" << "Camera gain bound min : " << m_MinGain << endl;
        LOG_DEBUG << "CameraGigeAravis::grabInitialization;" << "Camera gain bound max : " << m_MaxGain << endl;

        LOG_DEBUG << "CameraGigeAravis::grabInitialization;" << "Camera frame rate : " << m_FPS<<endl;

        capsString = arv_pixel_format_to_gst_caps_string(pixFormat);
        LOG_DEBUG << "Camera format : " << capsString << endl;

        m_Gain = arv_camera_get_gain(camera,&error);
        LOG_DEBUG << "Camera gain : " << m_Gain << endl;

        m_ExposureTime = arv_camera_get_exposure_time(camera, &error);

        LOG_DEBUG << "Camera exposure : " << m_ExposureTime << endl;
      
        LOG_DEBUG << "DEVICE SELECTED : " << arv_camera_get_device_id(camera,&error) << endl;

        LOG_DEBUG << "DEVICE NAME     : " << arv_camera_get_model_name(camera,&error) << endl;

        LOG_DEBUG << "DEVICE VENDOR   : " << arv_camera_get_vendor_name(camera,&error) << endl;

        LOG_DEBUG << "PAYLOAD         : " << payload << endl;
        LOG_DEBUG << "Start X         : " << m_StartX
            << "Start Y         : " << m_StartY << endl;
        LOG_DEBUG << "Width           : " << m_Width                               
             << "Height          : " << m_Height << endl;
        LOG_DEBUG << "Exp Range       : [" << m_MinExposure << " - " << m_MaxExposure   << "]" << endl;
        LOG_DEBUG << "Exp             : " << m_ExposureTime;
        LOG_DEBUG << "Gain Range      : [" << m_MinGain        << " - " << m_MaxGain << "]" << endl;
        LOG_DEBUG << "Gain            : " << m_Gain << endl;
        LOG_DEBUG << "Fps             : " << m_FPS << endl;
        LOG_DEBUG << "Type            : " << capsString << endl;

        // Create a new stream object. Open stream on Camera.
        stream = arv_camera_create_stream(camera, nullptr, nullptr,&error);

        if(stream == nullptr){

           LOG_ERROR << "Fail to create stream with arv_camera_create_stream()" << endl;
            return false;

        }

        if (ARV_IS_GV_STREAM(stream)){

            bool            arv_option_auto_socket_buffer   = true;
            bool            arv_option_no_packet_resend     = true;
            unsigned int    arv_option_packet_timeout       = 20;
            unsigned int    arv_option_frame_retention      = 100;

            if(arv_option_auto_socket_buffer){

                g_object_set(stream,
                            // ARV_GV_STREAM_SOCKET_BUFFER_FIXED : socket buffer is set to a given fixed value.
                            // ARV_GV_STREAM_SOCKET_BUFFER_AUTO: socket buffer is set with respect to the payload size.
                            "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                            // Socket buffer size, in bytes.
                            // Allowed values: >= G_MAXULONG
                            // Default value: 0
                            "socket-buffer-size", 0, nullptr);

            }

            if(arv_option_no_packet_resend){

                // # packet-resend : Enables or disables the packet resend mechanism

                // If packet resend is disabled and a packet has been lost during transmission,
                // the grab result for the returned buffer holding the image will indicate that
                // the grab failed and the image will be incomplete.
                //
                // If packet resend is enabled and a packet has been lost during transmission,
                // a request is sent to the camera. If the camera still has the packet in its
                // buffer, it will resend the packet. If there are several lost packets in a
                // row, the resend requests will be combined.

                g_object_set(stream,
                            // ARV_GV_STREAM_PACKET_RESEND_NEVER: never request a packet resend
                            // ARV_GV_STREAM_PACKET_RESEND_ALWAYS: request a packet resend if a packet was missing
                            // Default value: ARV_GV_STREAM_PACKET_RESEND_ALWAYS
                            "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, nullptr);

            }

            g_object_set(stream,
                        // # packet-timeout

                        // The Packet Timeout parameter defines how long (in milliseconds) we will wait for
                        // the next expected packet before it initiates a resend request.

                        // Packet timeout, in µs.
                        // Allowed values: [1000,10000000]
                        // Default value: 40000
                        "packet-timeout",/* (unsigned) arv_option_packet_timeout * 1000*/(unsigned)40000,
                        // # frame-retention

                        // The Frame Retention parameter sets the timeout (in milliseconds) for the
                        // frame retention timer. Whenever detection of the leader is made for a frame,
                        // the frame retention timer starts. The timer resets after each packet in the
                        // frame is received and will timeout after the last packet is received. If the
                        // timer times out at any time before the last packet is received, the buffer for
                        // the frame will be released and will be indicated as an unsuccessful grab.

                        // Packet retention, in µs.
                        // Allowed values: [1000,10000000]
                        // Default value: 200000
                        "frame-retention", /*(unsigned) arv_option_frame_retention * 1000*/(unsigned) 200000,nullptr);

        }else
            return false;

        // Push 50 buffer in the stream input buffer queue.
        for (int i = 0; i < 50; i++)
            arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));

        return true;
    }

    void CameraGigeAravis::grabCleanse(){}

    bool CameraGigeAravis::acqStart()
    {
        LOG_DEBUG << "Set camera to CONTINUOUS MODE" << endl;
        arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS,&error);
        
        LOG_DEBUG << "Set camera TriggerMode to Off" << endl;
        arv_device_set_string_feature_value(arv_camera_get_device (camera), "TriggerMode" , "Off", &error);
        
        LOG_DEBUG << "Start acquisition on camera" << endl;
        arv_camera_start_acquisition(camera, &error);

        return true;
    }

    void CameraGigeAravis::acqStop()
    {
        arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

        //LOG_DEBUG << "Completed buffers = " << (unsigned long long) nbCompletedBuffers;
        //LOG_DEBUG << "Failures          = " << (unsigned long long) nbFailures;
        //LOG_DEBUG << "Underruns         = " << (unsigned long long) nbUnderruns ;

        LOG_DEBUG << "Completed buffers = " << (unsigned long long) nbCompletedBuffers << endl;
        LOG_DEBUG << "Failures          = " << (unsigned long long) nbFailures << endl;
        LOG_DEBUG << "Underruns         = " << (unsigned long long) nbUnderruns << endl;

        LOG_DEBUG << "Stopping acquisition..." << endl;
        arv_camera_stop_acquisition(camera, &error);
        
        LOG_DEBUG << "Acquisition stopped." << endl;

        LOG_DEBUG << "Unreferencing stream." << endl;
        g_object_unref(stream);
        stream = nullptr;

        LOG_DEBUG << "Unreferencing camera." << endl;
        g_object_unref(camera);
        camera = nullptr;

    }

    bool CameraGigeAravis::grabImage(shared_ptr<Frame> newFrame){
        ArvBuffer *arv_buffer;

        arv_buffer = arv_stream_timeout_pop_buffer(stream,2000000); //us

        char *buffer_data;
        size_t buffer_size;

        if(arv_buffer == nullptr){

            throw runtime_error("arv_buffer is nullptr");
            return false;
        }else{

            try{
                if ( arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS )
                {
                    //BOOST_LOG_SEV(logger, normal) << "Success to grab a frame.";

                    buffer_data = (char *) arv_buffer_get_data (arv_buffer, &buffer_size);
                    //Timestamping.
                    //string acquisitionDate = TimeDate::localDateTime(microsec_clock::universal_time(),"%Y:%m:%d:%H:%M:%S");
                    //BOOST_LOG_SEV(logger, normal) << "Date : " << acquisitionDate;
                    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
                    string acquisitionDate = to_iso_extended_string(time);
                    //BOOST_LOG_SEV(logger, normal) << "Date : " << acqDateInMicrosec;

                    cv::Mat image;
                    CamPixFmt imgDepth = CamPixFmt::MONO8;
                    int saturateVal = 0;

                    if(pixFormat == ARV_PIXEL_FORMAT_MONO_8)
                    {
                        //BOOST_LOG_SEV(logger, normal) << "Creating cv::Mat 8 bits ...";
                        image = cv::Mat(m_Height, m_Width, CV_8UC1, buffer_data);
                        imgDepth = CamPixFmt::MONO8;
                        saturateVal = 255;

                    }
                    else if(pixFormat == ARV_PIXEL_FORMAT_MONO_12)
                    {
                        //BOOST_LOG_SEV(logger, normal) << "Creating cv::Mat 16 bits ...";
                        image = cv::Mat(m_Height, m_Width, CV_16UC1, buffer_data);
                        imgDepth = CamPixFmt::MONO12;
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
                        //LOG_DEBUG << "> Time shift : " << t3;
                    }
                    //BOOST_LOG_SEV(logger, normal) << "Creating frame object ...";
                    newFrame = make_shared<Frame>(image, m_Gain, m_ExposureTime, acquisitionDate);
                    //BOOST_LOG_SEV(logger, normal) << "Setting date of frame ...";
                    //newFrame.setAcqDateMicro(acqDateInMicrosec);
                    //BOOST_LOG_SEV(logger, normal) << "Setting fps of frame ...";
                    newFrame->mFps = m_FPS;
                    newFrame->mFormat = imgDepth;
                    //BOOST_LOG_SEV(logger, normal) << "Setting saturated value of frame ...";
                    newFrame->mSaturatedValue = saturateVal;
                    newFrame->mFrameNumber = frameCounter;
                    
                    frameCounter++;

                    //BOOST_LOG_SEV(logger, normal) << "Re-pushing arv buffer in stream ...";
                    arv_stream_push_buffer(stream, arv_buffer);

                    return true;

                }else{

                    switch(arv_buffer_get_status(arv_buffer)){

                        case 0 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_SUCCESS : the buffer contains a valid image" << endl;
                            break;
                        case 1 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_CLEARED: the buffer is cleared" << endl;
                            break;
                        case 2 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_TIMEOUT: timeout was reached before all packets are received" << endl;
                            break;
                        case 3 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_MISSING_PACKETS: stream has missing packets" << endl;
                            break;
                        case 4 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_WRONG_PACKET_ID: stream has packet with wrong id" << endl;
                            break;
                        case 5 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_SIZE_MISMATCH: the received image didn't fit in the buffer data space" << endl;
                            break;
                        case 6 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_FILLING: the image is currently being filled" << endl;
                            break;
                        case 7 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_ABORTED: the filling was aborted before completion" << endl;
                            break;

                    }
                    
                    arv_stream_push_buffer(stream, arv_buffer);
                    
                    return false;
                }

            }catch(exception& e){
                LOG_ERROR << e.what();
                return false;

            }
        }
    }


    bool CameraGigeAravis::grabSingleImage(shared_ptr<Frame> frame)
    {
        GError* error;

        bool res = false;
        LOG_DEBUG << "ARAVIS CREATE DEVICE" << endl;
        if(!createDevice())
            return false;
        LOG_DEBUG << "ARAVIS DEVICE CREATED" << endl;

        if(!setPixelFormat(frame->mFormat))
            return false;
        LOG_DEBUG << "ARAVIS PIXEL FORMAT OK" << endl;
        if(!setExposureTime(frame->mExposure))
            return false;
        LOG_DEBUG << "ARAVIS EXP OK" << endl;
        if(!setGain(frame->mGain))
            return false;
        LOG_DEBUG << "ARAVIS GAIN OK" << endl;
        if(frame->mWidth > 0 && frame->mHeight > 0) {

            setFrameSize(frame->mStartX, frame->mStartY, frame->mWidth, frame->mHeight,1);
            //arv_camera_set_region(camera, frame.m_StartX, frame.m_StartY, frame.m_Width, frame.m_Height);
            //arv_camera_get_region (camera, nullptr, nullptr, &m_Width, &m_Height);

        }else{

            int sensor_width, sensor_height;

            arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);

            // Use maximum sensor size.
            arv_camera_set_region(camera, 0, 0,sensor_width,sensor_height, &error);
            arv_camera_get_region (camera, nullptr, nullptr, &m_Width, &m_Height, &error);
        }

        LOG_DEBUG << "ARAVIS FRAME OK" << endl;

        payload = arv_camera_get_payload (camera, &error);

        pixFormat = arv_camera_get_pixel_format (camera, &error);

        arv_camera_get_exposure_time_bounds (camera, &m_MinExposure, &m_MaxExposure, &error);

        arv_camera_get_gain_bounds (camera, &m_MinGain, &m_MaxGain, &error);

        arv_camera_set_frame_rate(camera, frame->mFps, &error); /* Regular captures */

        m_FPS = arv_camera_get_frame_rate(camera, &error);

        capsString = arv_pixel_format_to_gst_caps_string(pixFormat);

        m_Gain    = arv_camera_get_gain(camera, &error);
        m_ExposureTime     = arv_camera_get_exposure_time(camera, &error);


        LOG_DEBUG << "DEVICE SELECTED : " << arv_camera_get_device_id(camera, &error) << endl;
        LOG_DEBUG << "DEVICE NAME     : " << arv_camera_get_model_name(camera, &error) << endl;
        LOG_DEBUG << "DEVICE VENDOR   : " << arv_camera_get_vendor_name(camera, &error) << endl;
        LOG_DEBUG << "PAYLOAD         : " << payload << endl;
        LOG_DEBUG << "Start X         : " << m_StartX << "Start Y         : " << m_StartY << endl;
        LOG_DEBUG << "Width           : " << m_Width  << "Height          : " << m_Height << endl;
        LOG_DEBUG << "Exp Range       : [" << m_MinExposure    << " - " << m_MaxExposure   << "]" << endl;
        LOG_DEBUG << "Exp             : " << m_ExposureTime << endl;
        LOG_DEBUG << "Gain Range      : [" << m_MinGain        << " - " << m_MaxGain       << "]" << endl;
        LOG_DEBUG << "Gain            : " << m_Gain << endl;
        LOG_DEBUG << "Fps             : " << m_FPS << endl;
        LOG_DEBUG << "Type            : " << capsString << endl;

        
        if(arv_camera_is_gv_device (camera)) {

            // http://www.baslerweb.com/media/documents/AW00064902000%20Control%20Packet%20Timing%20With%20Delays.pdf
            // https://github.com/GNOME/aravis/blob/06ac777fc6d98783680340f1c3f3ea39d2780974/src/arvcamera.c

            // Configure the inter packet delay to insert between each packet for the current stream
            // channel. This can be used as a crude flow-control mechanism if the application or the network
            // infrastructure cannot keep up with the packets coming from the device.
            arv_camera_gv_set_packet_delay (camera, 4000, &error);

            // Specifies the stream packet size, in bytes, to send on the selected channel for a GVSP transmitter
            // or specifies the maximum packet size supported by a GVSP receiver.
            arv_camera_gv_set_packet_size (camera, 1444, &error);

        }

        // Create a new stream object. Open stream on Camera.
        stream = arv_camera_create_stream(camera, nullptr, nullptr, &error);

        if(stream != nullptr){

            if(ARV_IS_GV_STREAM(stream)){

                bool            arv_option_auto_socket_buffer   = true;
                bool            arv_option_no_packet_resend     = true;
                unsigned int    arv_option_packet_timeout       = 20;
                unsigned int    arv_option_frame_retention      = 100;

                if(arv_option_auto_socket_buffer){

                    g_object_set(stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0, nullptr);

                }

                if(arv_option_no_packet_resend){

                    g_object_set(stream, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, nullptr);

                }

                g_object_set(stream, "packet-timeout", (unsigned)40000, "frame-retention", (unsigned) 200000,nullptr);

            }

            // Push 50 buffer in the stream input buffer queue.
            arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));

            // Set acquisition mode to continuous.
            arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_SINGLE_FRAME, &error);

            // Very usefull to avoid arv buffer timeout status
            sleep(1);

            // Start acquisition.
            arv_camera_start_acquisition(camera, &error);

            // Get image buffer.
            ArvBuffer *arv_buffer = arv_stream_timeout_pop_buffer(stream, frame->mExposure + 5000000); //us

            char *buffer_data;
            size_t buffer_size;

            LOG_DEBUG << ">> Acquisition in progress... (Please wait)" << endl;

            if (arv_buffer != nullptr){

                if(arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS){

                    buffer_data = (char *) arv_buffer_get_data (arv_buffer, &buffer_size);

                    //Timestamping.
                    boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();

                    if(pixFormat == ARV_PIXEL_FORMAT_MONO_8){

                        cv::Mat image = cv::Mat(m_Height, m_Width, CV_8UC1, buffer_data);
                        image.copyTo(*frame->Image.get());

                    }else if(pixFormat == ARV_PIXEL_FORMAT_MONO_12){

                        // Unsigned short image.
                        cv::Mat image = cv::Mat(m_Height, m_Width, CV_16UC1, buffer_data);

                        // http://www.theimagingsource.com/en_US/support/documentation/icimagingcontrol-class/PixelformatY16.htm
                        // Some sensors only support 10-bit or 12-bit pixel data. In this case, the least significant bits are don't-care values.
                        if(shiftBitsImage){
                            unsigned short * p;
                            for(int i = 0; i < image.rows; i++){
                                p = image.ptr<unsigned short>(i);
                                for(int j = 0; j < image.cols; j++) p[j] = p[j] >> 4;
                            }
                        }

                        image.copyTo(*frame->Image.get());
                    }

                    frame->mDate = TimeDate::splitIsoExtendedDate(to_iso_extended_string(time));
                    frame->mFps = arv_camera_get_frame_rate(camera, &error);

                    res = true;

                }else{

                    switch(arv_buffer_get_status(arv_buffer)){

                        case 0 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_SUCCESS : the buffer contains a valid image" << endl;
                            break;

                        case 1 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_CLEARED: the buffer is cleared" << endl;
                            break;

                        case 2 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_TIMEOUT: timeout was reached before all packets are received" << endl;
                            break;

                        case 3 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_MISSING_PACKETS: stream has missing packets" << endl;
                            break;

                        case 4 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_WRONG_PACKET_ID: stream has packet with wrong id" << endl;
                            break;

                        case 5 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_SIZE_MISMATCH: the received image didn't fit in the buffer data space" << endl;
                            break;

                        case 6 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_FILLING: the image is currently being filled" << endl;
                            break;

                        case 7 :
                            LOG_DEBUG << "ARV_BUFFER_STATUS_ABORTED: the filling was aborted before completion" << endl;
                            break;
                    }

                    res = false;

                }

                arv_stream_push_buffer(stream, arv_buffer);
           } else {
                LOG_ERROR << "Fail to pop buffer from stream." << endl;
                res = false;
           }

            arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

            LOG_DEBUG << ">> Completed buffers = " << (unsigned long long) nbCompletedBuffers << endl;
            LOG_DEBUG << ">> Failures          = " << (unsigned long long) nbFailures << endl;
            //LOG_DEBUG << ">> Underruns         = " << (unsigned long long) nbUnderruns ;

            // Stop acquisition.
            arv_camera_stop_acquisition(camera, &error);

            g_object_unref(stream);
            stream = nullptr;

            g_object_unref(camera);
            camera = nullptr;

        }

        return res;

    }

    void CameraGigeAravis::saveGenicamXml(string p){

        const char *xml;

        size_t size;

        xml = arv_device_get_genicam_xml (arv_camera_get_device(camera), &size);

        if (xml != nullptr){

            ofstream infFile;
            string infFilePath = p + "genicam.xml";
            infFile.open(infFilePath.c_str());
            infFile << string ( xml, size );
            infFile.close();

        }

    }

    //https://github.com/GNOME/aravis/blob/b808d34691a18e51eee72d8cac6cfa522a945433/src/arvtool.c
    void CameraGigeAravis::getAvailablePixelFormats() {

        ArvGc *genicam;
        ArvDevice *device;
        ArvGcNode *node;

        if(camera != nullptr) {

            device = arv_camera_get_device(camera);
            genicam = arv_device_get_genicam(device);
            node = arv_gc_get_node(genicam, "PixelFormat");

            if (ARV_IS_GC_ENUMERATION (node)) {

                const GSList *childs;
                const GSList *iter;
                vector<string> pixfmt;

                LOG_DEBUG << ">> Device pixel formats :";

                childs = arv_gc_enumeration_get_entries (ARV_GC_ENUMERATION (node));
                for (iter = childs; iter != nullptr; iter = iter->next) {
                    if (arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (iter->data), nullptr)) {

                        if(arv_gc_feature_node_is_available (ARV_GC_FEATURE_NODE (iter->data), nullptr)) {

                            {
                                string fmt = string(arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE (iter->data)));
                                transform(fmt.begin(), fmt.end(),fmt.begin(), ::toupper);
                                pixfmt.push_back(fmt);
                                LOG_DEBUG << "- " << fmt;

                            }
                        }
                    }
                }

                // Compare found pixel formats to currently formats supported by freeture

                LOG_DEBUG  <<  ">> Available pixel formats :" << endl;
                EParser<CamPixFmt> fmt;

                for( int i = 0; i != pixfmt.size(); i++ ) {

                    if(fmt.isEnumValue(pixfmt.at(i))) {

                        LOG_DEBUG << "Found " << pixfmt.at(i) << endl;

                    }
                    else {
                        LOG_DEBUG << "Not Found " << pixfmt.at(i) << endl;
                    }

                }

            }else {

                LOG_DEBUG << ">> Available pixel formats not found.";

            }

            g_object_unref(device);

        }

    }

    void CameraGigeAravis::getExposureBounds(double &eMin, double &eMax)
    {
        GError* error = nullptr;

        double exposureMin = 0.0;
        double exposureMax = 0.0;

        arv_camera_get_exposure_time_bounds(camera, &exposureMin, &exposureMax, &error);

        eMin = exposureMin;
        eMax = exposureMax;
    }

    double CameraGigeAravis::getExposureTime()
    {
        GError* error=nullptr;
        double result = arv_camera_get_exposure_time(camera, &error);
        delete error;
        return result;
    }

    void CameraGigeAravis::getGainBounds(double &gMin, double &gMax)
    {
        GError* error=nullptr;

        double gainMin = 0.0;
        double gainMax = 0.0;

        arv_camera_get_gain_bounds(camera, &gainMin, &gainMax, &error);

        delete error;
        gMin = gainMin;
        gMax = gainMax;
    }

    bool CameraGigeAravis::getPixelFormat(CamPixFmt &format){
        GError* error = nullptr;

        ArvPixelFormat pixFormat = arv_camera_get_pixel_format(camera, &error);

        delete error;

        switch(pixFormat){

            case ARV_PIXEL_FORMAT_MONO_8 :

                format = CamPixFmt::MONO8;

                break;

            case ARV_PIXEL_FORMAT_MONO_12 :

                format = CamPixFmt::MONO12;

                break;

            default :

                return false;

                break;

        }

        return true;
    }


    bool CameraGigeAravis::getFrameSize(int &x, int &y, int &w, int &h) {
        GError* error = nullptr;

        if(camera != nullptr) {

            int ww = 0, hh = 0, xx = 0, yy = 0;
            arv_camera_get_region(camera, &x, &y, &w, &h,&error);
            x = xx;
            y = yy;
            w = ww;
            h = hh;

        }

        delete error;

        return false;

    }

    bool CameraGigeAravis::getFPS(double &value){
        GError* error = nullptr;

        if(camera != nullptr) {

            value = arv_camera_get_frame_rate(camera,&error);
            return true;
        }

        delete error;

        return false;
    }

    string CameraGigeAravis::getModelName(){
        GError* error = nullptr;

        string result =  arv_camera_get_model_name(camera,&error);
        delete error;

        return result;
    }

    bool CameraGigeAravis::setExposureTime(double val)
    {
        double expMin, expMax;

        arv_camera_get_exposure_time_bounds(camera, &expMin, &expMax, &error);

        if(camera != nullptr){

            if(val >= expMin && val <= expMax) {
                m_ExposureTime = val;
                arv_camera_set_exposure_time(camera, val, &error);
            } else {

                LOG_DEBUG << "> Exposure value (" << val << ") is not in range [ " << expMin << " - " << expMax << " ]" << endl;
                
                return false;

            }

            return true;

        }

        return false;
    }

    bool CameraGigeAravis::setGain(double val){

        double gMin, gMax;

        arv_camera_get_gain_bounds (camera, &gMin, &gMax, &error);
        
        if (camera != nullptr){

            if((double)val >= gMin && (double)val <= gMax){

                m_Gain = val;
                arv_camera_set_gain (camera, (double)val,&error);
            } else {
                LOG_ERROR << "> Gain value (" << val << ") is not in range [ " << gMin << " - " << gMax << " ]" << endl;
                return false;

            }

        return true;

        }

        return false;

    }

    bool CameraGigeAravis::setFPS(double fps){

        if (camera != nullptr){
            arv_camera_set_frame_rate(camera, fps, &error);
            
            double setfps = arv_camera_get_frame_rate(camera, &error);
            
            if (setfps!=fps) {
                LOG_WARNING << "> Frame rate value (" << fps << ") can't be set! Please check genicam features." << endl;
            }
            m_FPS = fps;

            return true;

        }

        return false;

    }

    bool CameraGigeAravis::setPixelFormat(CamPixFmt depth){

        if (camera != nullptr){

            switch(depth){

            case CamPixFmt::MONO8 :
                    {
                        arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_8,&error);
                    }
                    break;

                case CamPixFmt::MONO12 :
                    {
                        arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_12,&error);
                    }
                    break;
                case CamPixFmt::MONO16 :
                    {
                        arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_16,&error);
                    }
                    break;
            }

            return true;
        }

        return false;

    }

    bool CameraGigeAravis::setFrameSize(int startx, int starty, int width, int height, bool customSize) {

        if (camera != nullptr){
            if(customSize) {
          
                if (arv_device_get_feature(arv_camera_get_device(camera), "OffsetX")) {
                    LOG_DEBUG << "Starting from : " << startx << "," << starty << endl;
                } else {
                    LOG_WARNING << "OffsetX, OffsetY are not available: cannot set offset." << endl;
                }

                arv_camera_set_region(camera, startx, starty, width, height,&error);

                arv_camera_get_region (camera, &m_StartX, &m_StartY, &width, &height,&error);

            // Default is maximum size
            } else {

                int sensor_width, sensor_height;

                arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);
                LOG_DEBUG << "Camera sensor size : " << sensor_width << "x" << sensor_height << endl;
              
                arv_camera_get_region (camera, nullptr, nullptr, &width, &height,&error);
            }
            return true;
        }
        return false;
    }

    bool CameraGigeAravis::FirstInitializeCamera(string cPath)
    {
        LOG_DEBUG << "CameraGigeAravis::FirstInitializeCamera(\"" << cPath << "\")" << endl;
        if (camera != nullptr)
        {
            ArvDevice* arv_device = arv_camera_get_device(camera);
            //arv_device_set_boolean_feature_value(arv_device, "AcquisitionFrameRateEnable", true, &error);
            ifstream file(cPath);
            if(!file.is_open())
            {
                LOG_ERROR << "Error opening file " << cPath << endl;
                return false;
            }

            string line;

            
            while (getline(file, line))
            {
                arv_device = arv_camera_get_device(camera);
                vector<string> line_data;
                istringstream ss(line);
                string token;

                if(line.find('=') < 2) 
                {
                    LOG_DEBUG << "Line " << line << " cannot be parsed" << endl;
                    continue;
                }
                
                while (getline(ss, token, '='))
                {
                    line_data.push_back(token);
                }

                
                LOG_DEBUG << "Applining init " << line << endl;
                const char* feature = line_data[0].c_str(); 
                if(line_data[2] == "bool")
                {
                    bool value = line_data[1] == "true";
                    arv_device_set_boolean_feature_value(arv_device, feature, value, &error);
                    LOG_ERROR  <<  "ERROR" << endl;
                } else if(line_data[2] == "string")
                {
                    const char* value = line_data[1].c_str();
                    arv_device_set_string_feature_value(arv_device, feature, value, &error);
                    LOG_ERROR  <<  "ERROR" << endl;
                }
                else if(line_data[2] == "float")
                {
                    float value = stof(line_data[1]);
                    arv_device_set_float_feature_value(arv_device, feature, value, &error);
                    LOG_ERROR  <<  "ERROR" << endl;
                }
                this_thread::sleep_for(chrono::milliseconds(1500));
            }
            fstream f2;
            f2.open(FREETURE_CHECK_INIT_DONE, ios::out);
            f2.close();
            return true;
        }
        return false;
    }

    bool CameraGigeAravis::initSDK()
    {
        return true;
    }

    bool CameraGigeAravis::initOnce()
    {
        return true;
    }

    bool CameraGigeAravis::init()
    {
        return true;
    }

    void CameraGigeAravis::fetchBounds(parameters&)
    {

    }

    void CameraGigeAravis::configure(parameters&)
    {

    }

    bool CameraGigeAravis::configurationCheck(parameters&)
    {
        return true;
    }

    double CameraGigeAravis::getMinExposureTime()
    {
        return 30;
    }

    bool CameraGigeAravis::destroyDevice()
    {
        return true;
    }

#endif
#endif
