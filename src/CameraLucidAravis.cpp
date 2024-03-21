/**
* \file    CameraLucidAravis.cpp
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief   Use Aravis library to pilot Lucid Cameras
*
*/

#include "Commons.h"

#include "Logger.h"

#include <opencv2/opencv.hpp>
#include <boost/date_time.hpp>

#include "TimeDate.h"
#include "EParser.h"
#include "Frame.h"

using namespace freeture;
using namespace std;

#ifdef LINUX
#ifdef USE_ARAVIS

#include "CameraLucidAravis.h"

void CheckAravisError(GError** gError)
{
    if (gError != nullptr)
    {
        GError* gError_ptr = *gError;

        if (gError_ptr != nullptr)
        {
            GError& _error = *gError_ptr;
            std::ostringstream oss;
            oss << "GError - Domain: " << _error.domain << ", code: " << _error.code << ", message: " << _error.message;

            throw runtime_error(oss.str());
            delete gError_ptr;
            gError_ptr = nullptr;
        }
    }
}

CameraLucidAravis::CameraLucidAravis(CameraDescription description, cameraParam settings) :
    Camera(description, settings),
    error(nullptr),
    camera(nullptr),
    pixFormat(),
    stream(nullptr),
    payload(0),
    capsString(nullptr),
    shiftBitsImage(settings.SHIFT_BITS),
    nbCompletedBuffers(0),
    nbFailures(0),
    nbUnderruns(0),
    frameCounter(0)
{
    m_ExposureAvailable = true;
    m_GainAvailable = true;
}

CameraLucidAravis::~CameraLucidAravis()
{
    if (stream != nullptr)
        g_object_unref(stream);

    if (camera != nullptr)
    {
        LOG_DEBUG << "Unreferencing camera." << endl;
        g_object_unref(camera);
    }

}

bool CameraLucidAravis::createDevice()
{
    LOG_DEBUG << "CameraLucidAravis::createDevice" << endl;

    int id = m_CameraDescriptor.DeviceId;

    string deviceName;

    if (!getDeviceNameById(id, deviceName))
        return false;

    if (camera == nullptr)
    {
        LOG_DEBUG << "CameraLucidAravis::createDevice: Instancing arv_camera_new" << endl;
        camera = arv_camera_new(deviceName.c_str(), &error);

        CheckAravisError(&error);
    }

    if (camera == nullptr)
    {
        LOG_ERROR << "Fail to connect the camera." << endl;
        return false;
    }

    getFPSBounds(m_MinFPS,m_MaxFPS);
    setFPS(m_MinFPS);

    return true;
}

bool freeture::CameraLucidAravis::destroyDevice()
{

    return true;
}

bool CameraLucidAravis::recreateDevice(int id) {
    LOG_DEBUG << "CameraLucidAravis::createDevice" << endl;

    string deviceName;
    //free(camera);

    if (!getDeviceNameById(id, deviceName))
        return false;

    if (camera != nullptr)
    {
        LOG_DEBUG << "CameraLucidAravis::createDevice: Instancing arv_camera_new" << endl;
        camera = arv_camera_new(deviceName.c_str(), &error);
        CheckAravisError(&error);
    }

    if (camera == nullptr)
    {
        LOG_ERROR << "Fail to connect the camera." << endl;
        return false;
    }

    getFPSBounds(m_MinFPS, m_MaxFPS);
    setFPS(m_MinFPS);

    return true;
}

bool CameraLucidAravis::setSize(int startx, int starty, int width, int height, bool customSize) {
    LOG_DEBUG << "CameraLucidAravis::setSize" << startx << "," << starty << "" << width << "x" << height << endl;

    if (camera == nullptr) {
        LOG_DEBUG << "CAMERA IS nullptr" << endl;
        return false;
    }

    if (customSize) {
        LOG_DEBUG << "custom size is ok" << endl;

        arv_camera_set_region(camera, startx, starty, width, height, &error);
        CheckAravisError(&error);

        arv_camera_get_region(camera, &m_StartX, &m_StartY, &m_Width, &m_Height, &error);
        CheckAravisError(&error);

        LOG_DEBUG << "Camera region size :" << m_Width << "x" << m_Height << endl;
        if (arv_device_get_feature(arv_camera_get_device(camera), "OffsetX")) {
            LOG_DEBUG << "Starting from :" << m_StartX << "," << m_StartY << endl;
        }
        else {
            LOG_WARNING << "OffsetX, OffsetY are not available: cannot set offset." << endl;
        }
        CheckAravisError(&error);


        // Default is maximum size
    }
    else {
        LOG_DEBUG << "custom size is false" << endl;
        int sensor_width, sensor_height;

        arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);
        CheckAravisError(&error);

        LOG_DEBUG << "Camera sensor size :" << sensor_width << "x" << sensor_height << endl;

        arv_camera_set_region(camera, 0, 0, sensor_width, sensor_height, &error);
        CheckAravisError(&error);

        arv_camera_get_region(camera, nullptr, nullptr, &m_Width, &m_Height, &error);
        CheckAravisError(&error);
    }

    return true;

}

bool CameraLucidAravis::getDeviceNameById(int id, string& device)
{
    LOG_DEBUG << "CameraLucidAravis::getDeviceNameById [#" << id << "]" << endl;

    arv_update_device_list();

    int n_devices = arv_get_n_devices();

    for (int i = 0; i < n_devices; i++) {

        if (id == i) {

            device = arv_get_device_id(i);
            return true;

        }
    }
    CheckAravisError(&error);

    LOG_ERROR << "Fail to retrieve camera with this ID." << endl;
    return false;

}



bool CameraLucidAravis::grabInitialization()
{
    LOG_DEBUG << "CameraLucidAravis::grabInitialization" << endl;

    frameCounter = 0;

    payload = arv_camera_get_payload(camera, &error);
    CheckAravisError(&error);

    LOG_DEBUG << "Camera payload :" << payload << endl;

    pixFormat = arv_camera_get_pixel_format(camera, &error);
    CheckAravisError(&error);

    arv_camera_get_frame_rate_bounds(camera, &m_MinFPS, &m_MaxFPS, &error);
    CheckAravisError(&error);

    arv_camera_get_exposure_time_bounds(camera, &m_MinExposure, &m_MaxExposure, &error);
    CheckAravisError(&error);

    LOG_DEBUG << "Camera exposure bound min :" << m_MinExposure << endl;
    LOG_DEBUG << "Camera exposure bound max :" << m_MaxExposure << endl;

    arv_camera_get_gain_bounds(camera, &m_MinGain, &m_MaxGain, &error);
    CheckAravisError(&error);

    LOG_DEBUG << "Camera gain bound min :" << m_MinGain << endl;
    LOG_DEBUG << "Camera gain bound max :" << m_MaxGain << endl;
    LOG_DEBUG << "Camera frame rate :" << m_FPS << endl;

    capsString = arv_pixel_format_to_gst_caps_string(pixFormat);
    LOG_DEBUG << "Camera format :" << capsString << endl;

    m_Gain = arv_camera_get_gain(camera, &error);
    CheckAravisError(&error);

    LOG_DEBUG << "Camera gain :" << m_Gain << endl;

    m_ExposureTime = arv_camera_get_exposure_time(camera, &error);
    CheckAravisError(&error);

    LOG_DEBUG << "Camera exposure :" << m_ExposureTime << endl;


    LOG_DEBUG << "DEVICE SELECTED :" << arv_camera_get_device_id(camera, &error) << endl;
    CheckAravisError(&error);

    LOG_DEBUG << "DEVICE NAME     :" << arv_camera_get_model_name(camera, &error) << endl;
    CheckAravisError(&error);

    LOG_DEBUG << "DEVICE VENDOR   :" << arv_camera_get_vendor_name(camera, &error) << endl;
    CheckAravisError(&error);

    LOG_DEBUG << "PAYLOAD         :" << payload << endl;
    LOG_DEBUG << "Start X         :" << m_StartX << "Start Y:" << m_StartY << endl;
    LOG_DEBUG << "Width           :" << m_Width << "Height:" << m_Height << endl;
    LOG_DEBUG << "Exp Range       : [" << m_MinExposure << "-" << m_MaxExposure << "]" << endl;
    LOG_DEBUG << "Exp             :" << m_ExposureTime << endl;
    LOG_DEBUG << "Gain Range      : [" << m_MinGain << "-" << m_MaxGain << "]" << endl;
    LOG_DEBUG << "Gain            :" << m_Gain << endl;
    LOG_DEBUG << "Fps Range       : [" << m_MinFPS << "-" << m_MaxFPS << "]" << endl;
    LOG_DEBUG << "Fps             :" << m_FPS << endl;
    LOG_DEBUG << "Type            :" << capsString << endl;

    // Create a new stream object. Open stream on Camera.
    stream = arv_camera_create_stream(camera, nullptr, nullptr, &error);
    CheckAravisError(&error);

    if (stream == nullptr)
    {
        LOG_ERROR << "Fail to create stream with arv_camera_create_stream()" << endl;
        return false;

    }

    if (ARV_IS_GV_STREAM(stream)) {

        bool            arv_option_auto_socket_buffer = true;
        bool            arv_option_no_packet_resend = true;
        unsigned int    arv_option_packet_timeout = 20;
        unsigned int    arv_option_frame_retention = 100;

        if (arv_option_auto_socket_buffer) {

            g_object_set(stream,
                // ARV_GV_STREAM_SOCKET_BUFFER_FIXED : socket buffer is set to a given fixed value.
                // ARV_GV_STREAM_SOCKET_BUFFER_AUTO: socket buffer is set with respect to the payload size.
                "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                // Socket buffer size, in bytes.
                // Allowed values: >= G_MAXULONG
                // Default value: 0
                "socket-buffer-size", 0, nullptr);

        }

        if (arv_option_no_packet_resend) {

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
            "frame-retention", /*(unsigned) arv_option_frame_retention * 1000*/(unsigned)200000, nullptr);

    }
    else
        return false;

    // Push 50 buffer in the stream input buffer queue.
    for (int i = 0; i < 50; i++)
        arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));

    return true;
}

void CameraLucidAravis::grabCleanse() {
    LOG_DEBUG << "CameraLucidAravis::grabCleanse";
}

bool CameraLucidAravis::acqStart()
{
    LOG_DEBUG << "CameraLucidAravis::acqStart" << endl;

    LOG_DEBUG << "Set camera to CONTINUOUS MODE" << endl;
    arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS, &error);
    CheckAravisError(&error);

    LOG_DEBUG << "Set camera TriggerMode to Off" << endl;
    /* arv_device_set_string_feature_value(arv_camera_get_device (camera), "TriggerMode" , "Off", &error);
    CheckAravisError(&error); */

    LOG_DEBUG << "Start acquisition on camera" << endl;
    arv_camera_start_acquisition(camera, &error);
    CheckAravisError(&error);

    return true;
}

void CameraLucidAravis::acqStop()
{
    LOG_DEBUG << "CameraLucidAravis::acqStop" << endl;

    arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

    //LOG_DEBUG << "Completed buffers ="<< (unsigned long long) nbCompletedBuffers;
    //LOG_DEBUG << "Failures          ="<< (unsigned long long) nbFailures;
    //LOG_DEBUG << "Underruns         ="<< (unsigned long long) nbUnderruns;

    LOG_DEBUG << "Completed buffers =" << (unsigned long long) nbCompletedBuffers << endl;
    LOG_DEBUG << "Failures          =" << (unsigned long long) nbFailures << endl;
    LOG_DEBUG << "Underruns         =" << (unsigned long long) nbUnderruns << endl;

    LOG_DEBUG << "Stopping acquisition..." << endl;
    arv_camera_stop_acquisition(camera, &error);
    CheckAravisError(&error);

    LOG_DEBUG << "Acquisition stopped." << endl;

    LOG_DEBUG << "Unreferencing stream." << endl;
    g_object_unref(stream);
    stream = nullptr;

}

bool CameraLucidAravis::grabImage(shared_ptr<Frame> newFrame)
{
    //LOG_DEBUG << "CameraLucidAravis::grabImage";

    ArvBuffer* arv_buffer;
    //exp = arv_camera_get_exposure_time(camera);

    arv_buffer = arv_stream_timeout_pop_buffer(stream, 2000000); //us
    char* buffer_data;
    size_t buffer_size;

    if (arv_buffer == nullptr) {

        throw runtime_error("arv_buffer is nullptr");
        return false;

    }
    else {

        try {

            if (arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS)
            {

                //BOOST_LOG_SEV(logger, normal) << "Success to grab a frame.";

                buffer_data = (char*)arv_buffer_get_data(arv_buffer, &buffer_size);

                //Timestamping.
                //string acquisitionDate = TimeDate::localDateTime(microsec_clock::universal_time(),"%Y:%m:%d:%H:%M:%S");
                //BOOST_LOG_SEV(logger, normal) << "Date :"<< acquisitionDate;
                boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
                string acquisitionDate = to_iso_extended_string(time);
                //BOOST_LOG_SEV(logger, normal) << "Date :"<< acqDateInMicrosec;

                cv::Mat image;
                CamPixFmt imgDepth = CamPixFmt::MONO8;
                int saturateVal = 0;

                if (pixFormat == ARV_PIXEL_FORMAT_MONO_8)
                {

                    //BOOST_LOG_SEV(logger, normal) << "Creating cv::Mat 8 bits ...";
                    image = cv::Mat(m_Height, m_Width, CV_8UC1, buffer_data);
                    imgDepth = CamPixFmt::MONO8;
                    saturateVal = 255;

                }
                else if (pixFormat == ARV_PIXEL_FORMAT_MONO_12)
                {

                    //BOOST_LOG_SEV(logger, normal) << "Creating cv::Mat 16 bits ...";
                    image = cv::Mat(m_Height, m_Width, CV_16UC1, buffer_data);
                    imgDepth = CamPixFmt::MONO12;
                    saturateVal = 4095;


                    if (shiftBitsImage) {
                        unsigned short* p;

                        for (int i = 0; i < image.rows; i++) {
                            p = image.ptr<unsigned short>(i);
                            for (int j = 0; j < image.cols; j++)
                                p[j] = p[j] >> 4;
                        }
                    }
                }
                else if (pixFormat == ARV_PIXEL_FORMAT_MONO_16)
                {

                    image = cv::Mat(m_Height, m_Width, CV_16UC1, buffer_data);
                    imgDepth = CamPixFmt::MONO16;
                    saturateVal = 65535;
                }

                newFrame = make_shared<Frame>(image, m_Gain, m_ExposureTime, acquisitionDate);
                newFrame->mFps = m_FPS;
                newFrame->mFormat = imgDepth;
                //BOOST_LOG_SEV(logger, normal) << "Setting saturated value of frame ...";
                newFrame->mSaturatedValue = saturateVal;
                newFrame->mFrameNumber = frameCounter;
                frameCounter++;

                //BOOST_LOG_SEV(logger, normal) << "Re-pushing arv buffer in stream ...";
                arv_stream_push_buffer(stream, arv_buffer);

                return true;
            }
            else {

                switch (arv_buffer_get_status(arv_buffer)) {

                case 0:
                    LOG_DEBUG << "ARV_BUFFER_STATUS_SUCCESS : the buffer contains a valid image" << endl;
                    break;
                case 1:
                    LOG_DEBUG << "ARV_BUFFER_STATUS_CLEARED: the buffer is cleared" << endl;
                    break;
                case 2:
                    LOG_DEBUG << "ARV_BUFFER_STATUS_TIMEOUT: timeout was reached before all packets are received" << endl;
                    break;
                case 3:
                    LOG_DEBUG << "ARV_BUFFER_STATUS_MISSING_PACKETS: stream has missing packets" << endl;
                    break;
                case 4:
                    LOG_DEBUG << "ARV_BUFFER_STATUS_WRONG_PACKET_ID: stream has packet with wrong id" << endl;
                    break;
                case 5:
                    LOG_DEBUG << "ARV_BUFFER_STATUS_SIZE_MISMATCH: the received image didn't fit in the buffer data space" << endl;
                    break;
                case 6:
                    LOG_DEBUG << "ARV_BUFFER_STATUS_FILLING: the image is currently being filled" << endl;
                    break;
                case 7:
                    LOG_DEBUG << "ARV_BUFFER_STATUS_ABORTED: the filling was aborted before completion" << endl;
                    break;

                }
                arv_stream_push_buffer(stream, arv_buffer);

                return false;
            }
        }
        catch (exception& e) {
            LOG_DEBUG << "CameraLucidAravis::grabImage EXC" << endl;
            LOG_DEBUG << e.what() << endl;
            return false;

        }
    }
}

bool CameraLucidAravis::grabSingleImage(shared_ptr<Frame> frame)
{
    LOG_DEBUG << "CameraLucidAravis::grabSingleImage" << endl;
    auto arv_device = arv_camera_get_device(camera);

    bool res = false;

    if (frame->mWidth > 0 && frame->mHeight > 0)
    {

        setFrameSize(frame->mStartX, frame->mStartY, frame->mWidth, frame->mHeight, 1);
        //arv_camera_set_region(camera, frame.mStartX, frame.mStartY, frame.mWidth, frame.mHeight);
        //arv_camera_get_region (camera, nullptr, nullptr, &mWidth, &mHeight);
    }
    else
    {

        int sensor_width, sensor_height;

        arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);
        CheckAravisError(&error);

        // Use maximum sensor size.
        arv_camera_set_region(camera, 0, 0, sensor_width, sensor_height, &error);
        CheckAravisError(&error);

        arv_camera_get_region(camera, nullptr, nullptr, &m_Width, &m_Height, &error);
        CheckAravisError(&error);

    }


    payload = arv_camera_get_payload(camera, &error);
    CheckAravisError(&error);


    pixFormat = arv_camera_get_pixel_format(camera, &error);
    CheckAravisError(&error);


    arv_camera_get_exposure_time_bounds(camera, &m_MinExposure, &m_MaxExposure, &error);
    CheckAravisError(&error);


    arv_camera_get_gain_bounds(camera, &m_MinGain, &m_MaxGain, &error);
    CheckAravisError(&error);


    arv_camera_set_frame_rate(camera, frame->mFps, &error); /* Regular captures */
    CheckAravisError(&error);

    LOG_DEBUG << "==========================" << endl;

    m_LastTemperature = arv_device_get_float_feature_value(arv_device, "DeviceTemperature", &error);
    CheckAravisError(&error);

    m_FPS = arv_camera_get_frame_rate(camera, &error);
    CheckAravisError(&error);


    capsString = arv_pixel_format_to_gst_caps_string(pixFormat);


    m_Gain = arv_camera_get_gain(camera, &error);
    CheckAravisError(&error);

    m_ExposureTime = arv_camera_get_exposure_time(camera, &error);
    CheckAravisError(&error);

    LOG_DEBUG << "DEVICE SELECTED :" << arv_camera_get_device_id(camera, &error) << endl;
    CheckAravisError(&error);

    LOG_DEBUG << "DEVICE NAME     :" << arv_camera_get_model_name(camera, &error) << endl;
    CheckAravisError(&error);

    LOG_DEBUG << "DEVICE VENDOR   :" << arv_camera_get_vendor_name(camera, &error) << endl;
    CheckAravisError(&error);

    LOG_DEBUG << "DEVICE TEMP     :" << m_LastTemperature << endl;

    LOG_DEBUG << "PAYLOAD         :" << payload << endl;
    LOG_DEBUG << "Start X         :" << m_StartX << "Start Y         :" << m_StartY << endl;
    LOG_DEBUG << "Width           :" << m_Width << "Height          :" << m_Height << endl;
    LOG_DEBUG << "Exp Range       : [" << m_MinExposure << "-" << m_MaxExposure << "]" << endl;
    LOG_DEBUG << "Exp             :" << m_ExposureTime << endl;
    LOG_DEBUG << "Gain Range      : [" << m_MinGain << "-" << m_MaxGain << "]" << endl;
    LOG_DEBUG << "Gain            :" << m_Gain << endl;
    LOG_DEBUG << "Fps Range       : [" << m_MinFPS << "-" << m_MaxFPS << "]" << endl;
    LOG_DEBUG << "Fps             :" << m_FPS << endl;
    LOG_DEBUG << "Type            :" << capsString << endl;

    // Create a new stream object. Open stream on Camera.
    stream = arv_camera_create_stream(camera, nullptr, nullptr, &error);
    CheckAravisError(&error);

    if (stream != nullptr) {

        if (ARV_IS_GV_STREAM(stream)) {

            bool            arv_option_auto_socket_buffer = true;
            bool            arv_option_no_packet_resend = true;
            unsigned int    arv_option_packet_timeout = 20;
            unsigned int    arv_option_frame_retention = 100;

            if (arv_option_auto_socket_buffer) {

                g_object_set(stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0, nullptr);

            }

            if (arv_option_no_packet_resend) {

                g_object_set(stream, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, nullptr);

            }

            g_object_set(stream, "packet-timeout", (unsigned)40000, "frame-retention", (unsigned)200000, nullptr);

        }

        // Push 50 buffer in the stream input buffer queue.
        arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));


        // Set acquisition mode to continuous.
        arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_SINGLE_FRAME, &error);


        CheckAravisError(&error);

        // Very usefull to avoid arv buffer timeout status
        sleep(1);

        // Start acquisition.
        arv_camera_start_acquisition(camera, &error);
        CheckAravisError(&error);

        // Get image buffer.
        ArvBuffer* arv_buffer = arv_stream_timeout_pop_buffer(stream, frame->mExposure + 5000000); //us




        char* buffer_data;
        size_t buffer_size;

        LOG_DEBUG << "Acquisition in progress... (Please wait)";

        if (arv_buffer != nullptr)
        {

            if (arv_buffer_get_status(arv_buffer) == ARV_BUFFER_STATUS_SUCCESS) {

                buffer_data = (char*)arv_buffer_get_data(arv_buffer, &buffer_size);

                //Timestamping.
                boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();

                if (pixFormat == ARV_PIXEL_FORMAT_MONO_8) {

                    cv::Mat image = cv::Mat(m_Height, m_Width, CV_8UC1, buffer_data);
                    image.copyTo(*frame->Image.get());

                }
                else if (pixFormat == ARV_PIXEL_FORMAT_MONO_12 || pixFormat == ARV_PIXEL_FORMAT_MONO_16) {

                    // Unsigned short image.
                    cv::Mat image = cv::Mat(m_Height, m_Width, CV_16UC1, buffer_data);

                    // http://www.theimagingsource.com/en_US/support/documentation/icimagingcontrol-class/PixelformatY16.htm
                    // Some sensors only support 10-bit or 12-bit pixel data. In this case, the least significant bits are don't-care values.
                    if (shiftBitsImage && pixFormat != ARV_PIXEL_FORMAT_MONO_16) {
                        unsigned short* p;
                        for (int i = 0; i < image.rows; i++) {
                            p = image.ptr<unsigned short>(i);
                            for (int j = 0; j < image.cols; j++) p[j] = p[j] >> 4;
                        }
                    }

                    image.copyTo(*frame->Image.get());
                }

                frame->mDate = TimeDate::splitIsoExtendedDate(to_iso_extended_string(time));
                frame->mFps = arv_camera_get_frame_rate(camera, &error);
                CheckAravisError(&error);

                res = true;

            }
            else {

                switch (arv_buffer_get_status(arv_buffer)) {

                case 0:

                    LOG_DEBUG << "ARV_BUFFER_STATUS_SUCCESS : the buffer contains a valid image" << endl;

                    break;

                case 1:

                    LOG_DEBUG << "ARV_BUFFER_STATUS_CLEARED: the buffer is cleared" << endl;

                    break;

                case 2:

                    LOG_DEBUG << "ARV_BUFFER_STATUS_TIMEOUT: timeout was reached before all packets are received" << endl;

                    break;

                case 3:

                    LOG_DEBUG << "ARV_BUFFER_STATUS_MISSING_PACKETS: stream has missing packets" << endl;

                    break;

                case 4:

                    LOG_DEBUG << "ARV_BUFFER_STATUS_WRONG_PACKET_ID: stream has packet with wrong id" << endl;

                    break;

                case 5:

                    LOG_DEBUG << "ARV_BUFFER_STATUS_SIZE_MISMATCH: the received image didn't fit in the buffer data space" << endl;

                    break;

                case 6:

                    LOG_DEBUG << "ARV_BUFFER_STATUS_FILLING: the image is currently being filled" << endl;

                    break;

                case 7:

                    LOG_DEBUG << "ARV_BUFFER_STATUS_ABORTED: the filling was aborted before completion" << endl;

                    break;


                }

                res = false;

            }

            arv_stream_push_buffer(stream, arv_buffer);

        }
        else {

            LOG_ERROR << "Fail to pop buffer from stream." << endl;
            res = false;
        }

        arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

        LOG_DEBUG << "Completed buffers =" << (unsigned long long) nbCompletedBuffers << endl;
        LOG_DEBUG << "Failures          =" << (unsigned long long) nbFailures << endl;
        //LOG_DEBUG << "Underruns         ="<< (unsigned long long) nbUnderruns;

        // Stop acquisition.
        arv_camera_stop_acquisition(camera, &error);
        CheckAravisError(&error);


        /* g_object_unref(stream);
        stream = nullptr;
        g_object_unref(camera);
        camera = nullptr; */

    }

    return res;

}

void CameraLucidAravis::saveGenicamXml(string p) {
    LOG_DEBUG << "CameraLucidAravis::saveGenicamXml" << endl;

    const char* xml;

    size_t size;

    xml = arv_device_get_genicam_xml(arv_camera_get_device(camera), &size);

    if (xml != nullptr) {
        ofstream infFile;
        string infFilePath = p + "genicam.xml";
        infFile.open(infFilePath.c_str());
        infFile << string(xml, size);
        infFile.close();
    }
}

    //https://github.com/GNOME/aravis/blob/b808d34691a18e51eee72d8cac6cfa522a945433/src/arvtool.c
void CameraLucidAravis::getAvailablePixelFormats() {
    LOG_DEBUG << "CameraLucidAravis::getAvailablePixelFormats" << endl;

    ArvGc* genicam;
    ArvDevice* device;
    ArvGcNode* node;

    if (camera != nullptr) {

        device = arv_camera_get_device(camera);
        genicam = arv_device_get_genicam(device);
        node = arv_gc_get_node(genicam, "PixelFormat");

        if (ARV_IS_GC_ENUMERATION(node)) {

            const GSList* childs;
            const GSList* iter;
            vector<string> pixfmt;

            LOG_DEBUG << "Device pixel formats :" << endl;

            childs = arv_gc_enumeration_get_entries(ARV_GC_ENUMERATION(node));
            for (iter = childs; iter != nullptr; iter = iter->next) {
                if (arv_gc_feature_node_is_implemented(ARV_GC_FEATURE_NODE(iter->data), nullptr)) {

                    if (arv_gc_feature_node_is_available(ARV_GC_FEATURE_NODE(iter->data), nullptr)) {

                        {
                            string fmt = string(arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(iter->data)));
                            transform(fmt.begin(), fmt.end(), fmt.begin(), ::toupper);
                            pixfmt.push_back(fmt);
                            LOG_DEBUG << "-" << fmt << endl;

                        }
                    }
                }
            }

            // Compare found pixel formats to currently formats supported by freeture

            LOG_DEBUG << "Available pixel formats :" << endl;
            EParser<CamPixFmt> fmt;

            for (int i = 0; i != pixfmt.size(); i++) {

                if (fmt.isEnumValue(pixfmt.at(i))) {

                    LOG_DEBUG << "Found " << pixfmt.at(i) << endl;

                }
                else
                    LOG_WARNING << "NOT Found " << pixfmt.at(i) << endl;

            }

        }
        else {

            LOG_DEBUG << "Available pixel formats not found.";

        }

        g_object_unref(device);

    }

}



    void CameraLucidAravis::getFPSBounds(double &fMin, double &fMax)
    {
        LOG_DEBUG << "CameraLucidAravis::getFPSBounds " << endl;
        
        
        if (camera == nullptr) {
            LOG_DEBUG << "CAMERA IS nullptr" << endl;
        }


        double fpsMin = 0.0;
        double fpsMax = 0.0;

        arv_camera_get_frame_rate_bounds(camera, &fpsMin, &fpsMax, &error);
        
        CheckAravisError(&error);
        
        fMin = fpsMin;
        fMax = fpsMax;

        LOG_DEBUG <<"["<< fpsMin<<", "<< fpsMax<<"]" << endl;
    }



    void CameraLucidAravis::getExposureBounds(double &eMin, double &eMax)
    {

        LOG_DEBUG << "CameraLucidAravis::getExposureBounds " << endl;

        double exposureMin = 0.0;
        double exposureMax = 0.0;

        if (arv_camera_get_exposure_time_auto(camera, &error)) {
            LOG_DEBUG << "CAMERA EXP IS SET TO AUTO" << endl;
        }

        arv_camera_get_exposure_time_bounds(camera, &exposureMin, &exposureMax, &error);
        CheckAravisError(&error);

        eMin = exposureMin;
        eMax = exposureMax;
        LOG_DEBUG <<"["<< exposureMin<<", "<< exposureMax<<"]" << endl;
    }

    double CameraLucidAravis::getExposureTime()
    {
        LOG_DEBUG << "CameraLucidAravis::getExposureTime" << endl;

        double result = arv_camera_get_exposure_time(camera, &error);
        CheckAravisError(&error);
        return result;
    }

    void CameraLucidAravis::getGainBounds(double &gMin, double &gMax){
        LOG_DEBUG << "CameraLucidAravis::getGainBounds" << endl;


        double gainMin = 0.0;
        double gainMax = 0.0;

        arv_camera_get_gain_bounds(camera, &gainMin, &gainMax, &error);
        CheckAravisError(&error);

        gMin = gainMin;
        gMax = gainMax;

    }

    bool CameraLucidAravis::getPixelFormat(CamPixFmt &format){
        LOG_DEBUG << "CameraLucidAravis::getPixelFormat" << endl;

        ArvPixelFormat pixFormat = arv_camera_get_pixel_format(camera, &error);
        CheckAravisError(&error);

        switch(pixFormat){

            case ARV_PIXEL_FORMAT_MONO_8 :

                format = CamPixFmt::MONO8;

                break;

            case ARV_PIXEL_FORMAT_MONO_12 :

                format = CamPixFmt::MONO12;

                break;
           case ARV_PIXEL_FORMAT_MONO_16 :

                format = CamPixFmt::MONO16;

                break;

            default :

                return false;

                break;

        }

        return true;
    }


    bool CameraLucidAravis::getFrameSize(int &x, int &y, int &w, int &h) {
        LOG_DEBUG << "CameraLucidAravis::getFrameSize" << endl;

        if(camera != nullptr) {

            int ww = 0, hh = 0, xx = 0, yy = 0;
            arv_camera_get_region(camera, &x, &y, &w, &h,&error);
            CheckAravisError(&error);
            x = xx;
            y = yy;
            w = ww;
            h = hh;

        }

        return false;

    }

    bool CameraLucidAravis::getFPS(double &value){
        LOG_DEBUG << "CameraLucidAravis::getFPS" << endl;

        if(camera != nullptr) {

            value = arv_camera_get_frame_rate(camera,&error);
            CheckAravisError(&error);

            return true;

        }

        return false;
    }

    string CameraLucidAravis::getModelName(){
        LOG_DEBUG << "CameraLucidAravis::getModelName" << endl;
        

        string result =  arv_camera_get_model_name(camera,&error);
        CheckAravisError(&error);

        return result;
    }

    bool CameraLucidAravis::setExposureTime(double val)
    {
        LOG_DEBUG << "CameraLucidAravis::setExposureTime ["<< val<<"]" << endl;
        if (camera == nullptr) {
            LOG_DEBUG << "CAMERA IS nullptr" << endl;
            return false;
        }

        double expMin, expMax;

        arv_camera_get_exposure_time_bounds(camera, &expMin, &expMax,&error);
        CheckAravisError(&error);

        LOG_DEBUG << "arv_camera_get_exposure_time_bounds ["<< expMin<< ", "<< expMax<<"]" << endl;

        if(camera != nullptr) {


            if(val >= expMin && val <= expMax) {

                m_ExposureTime = val;
                arv_camera_set_exposure_time(camera, val, &error);
                CheckAravisError(&error);

                double min, max;
                getFPSBounds(min,max);

            } else {

                LOG_DEBUG << "Exposure value (" << val << ") is not in range ["<< expMin <<"-"<< expMax <<"]" << endl;
                if(val < expMin) {
                    LOG_DEBUG << "Exposure value (" << val << ") less" << endl;
                }
                if(val > expMax) {
                    LOG_DEBUG << "Exposure value (" << val << ") bigger" << endl;
                }
                return false;

            }

            return true;

        }

        return false;
    }

    bool CameraLucidAravis::setAutoExposure(bool val)
    {
        LOG_DEBUG << "CameraLucidAravis::setAutoExp" << endl;
        if (camera == nullptr) {
            LOG_DEBUG << "CAMERA IS nullptr" << endl;
            return false;
        }

        ArvAuto arv_mode = val ? ARV_AUTO_ONCE : ARV_AUTO_OFF;
        arv_camera_set_exposure_time_auto(camera, arv_mode, &error);
        
        CheckAravisError(&error);

        return true;
    }

    bool CameraLucidAravis::setGain(double val) {
        LOG_DEBUG << "CameraLucidAravis::setGain" << endl;
        if (camera == nullptr) {
            LOG_DEBUG << "CAMERA IS nullptr" << endl;
            return false;
        }

        double gMin, gMax;

        arv_camera_get_gain_bounds(camera, &gMin, &gMax, &error);
        CheckAravisError(&error);

        if (camera != nullptr) {

            if ((double)val >= gMin && (double)val <= gMax) {

                m_Gain = val;
                arv_camera_set_gain(camera, (double)val, &error);
                CheckAravisError(&error);

            }
            else {

                LOG_DEBUG << "Gain value (" << val << ") is not in range [" << gMin << "-" << gMax << "]" << endl;
                LOG_ERROR << "Gain value (" << val << ") is not in range [" << gMin << "-" << gMax << "]" << endl;
                return false;

            }

            return true;

        }

        return false;

    }

    bool CameraLucidAravis::setFPS(double fps)
    {
        LOG_DEBUG << "CameraLucidAravis::setFPS" << " [" << fps << "]" << endl;
        if (camera == nullptr) {
            LOG_DEBUG << "CAMERA IS nullptr" << endl;
            return false;
        }

        /* //ADDED FOR DEBUG
        double minFps,maxFps;
        getFPSBounds(minFps,maxFps); */

        //NEED TO ENABLE FRAME RATE  SETUP
        arv_device_set_boolean_feature_value(arv_camera_get_device(camera), "AcquisitionFrameRateEnable", true, &error);
        CheckAravisError(&error);

        arv_camera_set_frame_rate(camera, fps, &error);
        CheckAravisError(&error);

        double setfps = arv_camera_get_frame_rate(camera, &error);
        CheckAravisError(&error);
        LOG_DEBUG << "==" << setfps << endl;

        int fps_test_l = (int)setfps;
        int fps_test_r = (int)fps;

        if (fps_test_l != fps_test_r) {
            LOG_DEBUG << "Frame rate value (" << fps << ") can't be set! Please check genicam features." << endl;
            LOG_WARNING << "Frame rate value (" << fps << ") can't be set!" << endl;
        }

        return true;
    }

    

    bool CameraLucidAravis::setPixelFormat(CamPixFmt depth) {
        LOG_DEBUG << "CameraLucidAravis::setPixelFormat" << endl;
        if (camera == nullptr) {
            LOG_DEBUG << "CAMERA IS nullptr" << endl;
            return false;
        }

        switch (depth) {

        case CamPixFmt::MONO8:
        {
            LOG_DEBUG << "SET PIXEL FORMAT TO MONO_8" << endl;
            arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_8, &error);
            CheckAravisError(&error);
        }
        break;

        case CamPixFmt::MONO12:
        {
            LOG_DEBUG << "SET PIXEL FORMAT TO MONO_12" << endl;
            arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_12, &error);
            CheckAravisError(&error);
        }
        break;
        case CamPixFmt::MONO16:
        {
            LOG_DEBUG << "SET PIXEL FORMAT TO MONO_16" << endl;
            arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_16, &error);
            CheckAravisError(&error);
        }
        break;
        }

        return true;
    }

    bool CameraLucidAravis::setFrameSize(int startx, int starty, int width, int height, bool customSize) {
        LOG_DEBUG << "CameraLucidAravis::setFrameSize" << endl;
        if (camera == nullptr) {
            LOG_DEBUG << "CAMERA IS nullptr" << endl;
            return false;
        }

        if (customSize) {

            if (arv_device_get_feature(arv_camera_get_device(camera), "OffsetX")) {
                LOG_DEBUG << "Starting from :" << m_StartX << "," << m_StartY << endl;
                LOG_DEBUG << "Starting from :" << m_StartX << "," << m_StartY << endl;
            }
            else {
                LOG_WARNING << "OffsetX, OffsetY are not available: cannot set offset." << endl;
            }

            arv_camera_set_region(camera, startx, starty, width, height, &error);
            CheckAravisError(&error);

            arv_camera_get_region(camera, &m_StartX, &m_StartY, &m_Width, &m_Height, &error);
            CheckAravisError(&error);

            // Default is maximum size
        }
        else {

            int sensor_width, sensor_height;

            arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);
            CheckAravisError(&error);

            LOG_DEBUG << "Camera sensor size :" << sensor_width << "x" << sensor_height << endl;

            arv_camera_set_region(camera, 0, 0, sensor_width, sensor_height, &error);
            CheckAravisError(&error);

            arv_camera_get_region(camera, nullptr, nullptr, &m_Width, &m_Height, &error);
            CheckAravisError(&error);
        }

        return true;
    }

    bool CameraLucidAravis::initSDK()
    {
        return true;
    }

    bool CameraLucidAravis::initOnce()
    {
        return true;
    }

    bool CameraLucidAravis::init()
    {
        return true;
    }

    void CameraLucidAravis::fetchBounds(parameters&)
    {

    }

    void CameraLucidAravis::configure(parameters&)
    {

    }

    bool CameraLucidAravis::configurationCheck(parameters&)
    {
        return true;
    }

    double CameraLucidAravis::getMinExposureTime()
    {
        return 31;
    }

#endif
#endif
