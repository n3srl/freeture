/**
* \file    CameraLucidArena.cpp
* \author  Andrea Novati -- N3 S.r.l.
* \version 1.2
* \date    03/15/2023
* \brief   Use Aravis library to pilot Lucid Cameras
*
*/

#include "ErrorManager.cpp"
#include "CameraLucidArena.h"

#ifdef LINUX

    boost::log::sources::severity_logger< LogSeverityLevel >  CameraLucidArena::logger;
    CameraLucidArena::Init CameraLucidArena::initializer;

    CameraLucidArena::CameraLucidArena(bool shift):
    camera(NULL), mStartX(0), mStartY(0), mWidth(0), mHeight(0), fps(0), gainMin(0.0), gainMax(0.0),
    payload(0), exposureMin(0), exposureMax(0), gain(0), exp(0), nbCompletedBuffers(0),
    nbFailures(0), nbUnderruns(0), frameCounter(0), shiftBitsImage(shift), stream(NULL) {
        mExposureAvailable = true;
        mGainAvailable = true;
        mInputDeviceType = CAMERA;
    }

    CameraLucidArena::CameraLucidArena():
    camera(NULL), mStartX(0), mStartY(0), mWidth(0), mHeight(0), fps(0), gainMin(0.0), gainMax(0.0),
    payload(0), exposureMin(0), exposureMax(0), gain(0), exp(0), nbCompletedBuffers(0),
    nbFailures(0), nbUnderruns(0), frameCounter(0), shiftBitsImage(false), stream(NULL) {
        mExposureAvailable = true;
        mGainAvailable = true;
        mInputDeviceType = CAMERA;
    }

    CameraLucidArena::~CameraLucidArena()
    {


        if(stream != NULL)
            g_object_unref(stream);

        if(camera != NULL)
        {
            BOOST_LOG_SEV(logger, notification) << "Unreferencing camera.";
            g_object_unref(camera);
        }

    }

    bool CameraLucidArena::createDevice(int id){
       cout << "CameraLucidArena::createDevice"<< endl;

        string deviceName;

        if(!getDeviceNameById(id, deviceName))
            return false;

        if (camera == nullptr)
        {
            cout << "CameraLucidArena::createDevice: Instancing arv_camera_new"<<endl;
            camera = arv_camera_new(deviceName.c_str(),&error);
            ErrorManager::CheckAravisError(&error);
        }

        if(camera == nullptr)
        {
            BOOST_LOG_SEV(logger, fail) << "Fail to connect the camera.";
            return false;
        }

        getFPSBounds(fpsMin,fpsMax);
        setFPS(fpsMin);

        return true;
    }

    bool CameraLucidArena::setSize(int startx, int starty, int width, int height, bool customSize) {
        cout << "CameraLucidArena::setSize"<< endl;

        if(customSize) {


            arv_camera_set_region(camera, startx, starty, width, height,&error);
            ErrorManager::CheckAravisError(&error);

            arv_camera_get_region (camera, &mStartX, &mStartY, &mWidth, &mHeight,&error);
            ErrorManager::CheckAravisError(&error);

            BOOST_LOG_SEV(logger, notification) << "Camera region size : " << mWidth << "x" << mHeight;
            if (arv_device_get_feature(arv_camera_get_device(camera), "OffsetX")) {
                BOOST_LOG_SEV(logger, notification) << "Starting from : " << mStartX << "," << mStartY;
            } else {
                BOOST_LOG_SEV(logger, warning) << "OffsetX, OffsetY are not available: cannot set offset.";
            }


        // Default is maximum size
        } else {

            int sensor_width, sensor_height;

            arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);
            ErrorManager::CheckAravisError(&error);

            BOOST_LOG_SEV(logger, notification) << "Camera sensor size : " << sensor_width << "x" << sensor_height;

            arv_camera_set_region(camera, 0, 0,sensor_width,sensor_height,&error);
            ErrorManager::CheckAravisError(&error);

            arv_camera_get_region (camera, NULL, NULL, &mWidth, &mHeight,&error);
            ErrorManager::CheckAravisError(&error);
        }

        return true;

    }

    bool CameraLucidArena::getDeviceNameById(int id, string &device)
    {
        cout << "CameraLucidArena::getDeviceNameById [#"<< id<<"]"<< endl;

        arv_update_device_list();

        int n_devices = arv_get_n_devices();

        for(int i = 0; i< n_devices; i++){

            if(id == i){

                device = arv_get_device_id(i);
                return true;

            }
        }

        BOOST_LOG_SEV(logger, fail) << "Fail to retrieve camera with this ID.";
        return false;

    }



    bool CameraLucidArena::grabInitialization()
    {
        cout << "CameraLucidArena::grabInitialization"<< endl;

        frameCounter = 0;

        payload = arv_camera_get_payload (camera, &error);
        ErrorManager::CheckAravisError(&error);

        BOOST_LOG_SEV(logger, notification) << "Camera payload : " << payload;

        pixFormat = arv_camera_get_pixel_format(camera, &error);
        ErrorManager::CheckAravisError(&error);

        arv_camera_get_frame_rate_bounds(camera, &fpsMin, &fpsMax, &error);
        ErrorManager::CheckAravisError(&error);

        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax, &error);
        ErrorManager::CheckAravisError(&error);

        BOOST_LOG_SEV(logger, notification) << "Camera exposure bound min : " << exposureMin;
        BOOST_LOG_SEV(logger, notification) << "Camera exposure bound max : " << exposureMax;

        arv_camera_get_gain_bounds (camera, &gainMin, &gainMax, &error);
        ErrorManager::CheckAravisError(&error);

        BOOST_LOG_SEV(logger, notification) << "Camera gain bound min : " << gainMin;
        BOOST_LOG_SEV(logger, notification) << "Camera gain bound max : " << gainMax;

        BOOST_LOG_SEV(logger, notification) << "Camera frame rate : " << fps;

        capsString = arv_pixel_format_to_gst_caps_string(pixFormat);
        BOOST_LOG_SEV(logger, notification) << "Camera format : " << capsString;

        gain = arv_camera_get_gain(camera,&error);
        ErrorManager::CheckAravisError(&error);

        BOOST_LOG_SEV(logger, notification) << "Camera gain : " << gain;

        exp = arv_camera_get_exposure_time(camera, &error);
        ErrorManager::CheckAravisError(&error);

        BOOST_LOG_SEV(logger, notification) << "Camera exposure : " << exp;

        cout << endl;

        cout << "DEVICE SELECTED : " << arv_camera_get_device_id(camera,&error)    << endl;
        ErrorManager::CheckAravisError(&error);

        cout << "DEVICE NAME     : " << arv_camera_get_model_name(camera,&error)   << endl;
        ErrorManager::CheckAravisError(&error);

        cout << "DEVICE VENDOR   : " << arv_camera_get_vendor_name(camera,&error)  << endl;
        ErrorManager::CheckAravisError(&error);

        cout << "PAYLOAD         : " << payload                             << endl;
        cout << "Start X         : " << mStartX                             << endl
             << "Start Y         : " << mStartY                             << endl;
        cout << "Width           : " << mWidth                               << endl
             << "Height          : " << mHeight                              << endl;
        cout << "Exp Range       : [" << exposureMin    << " - " << exposureMax   << "]"  << endl;
        cout << "Exp             : " << exp                                 << endl;
        cout << "Gain Range      : [" << gainMin        << " - " << gainMax       << "]"  << endl;
        cout << "Gain            : " << gain                                << endl;
        cout << "Fps Range       : [" << fpsMin    << " - " << fpsMax   << "]"  << endl;
        cout << "Fps             : " << fps                                 << endl;
        cout << "Type            : " << capsString                         << endl;

        cout << endl;

        // Create a new stream object. Open stream on Camera.
        stream = arv_camera_create_stream(camera, NULL, NULL,&error);
        ErrorManager::CheckAravisError(&error);

        if(stream == NULL){

            BOOST_LOG_SEV(logger, critical) << "Fail to create stream with arv_camera_create_stream()";
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
                            "socket-buffer-size", 0, NULL);

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
                            "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, NULL);

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
                        "frame-retention", /*(unsigned) arv_option_frame_retention * 1000*/(unsigned) 200000,NULL);

        }else
            return false;

        // Push 50 buffer in the stream input buffer queue.
        for (int i = 0; i < 50; i++)
            arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));



        return true;
    }

    void CameraLucidArena::grabCleanse(){
       cout << "CameraLucidArena::grabCleanse"<< endl;
    }

    bool CameraLucidArena::acqStart()
    {
        cout << "CameraLucidArena::acqStart"<< endl;

        BOOST_LOG_SEV(logger, notification) << "Set camera to CONTINUOUS MODE";
        arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS, &error);
        ErrorManager::CheckAravisError(&error);

        BOOST_LOG_SEV(logger, notification) << "Set camera TriggerMode to Off";
        arv_device_set_string_feature_value(arv_camera_get_device (camera), "TriggerMode" , "Off", &error);
        ErrorManager::CheckAravisError(&error);

        BOOST_LOG_SEV(logger, notification) << "Start acquisition on camera";
        arv_camera_start_acquisition(camera, &error);
        ErrorManager::CheckAravisError(&error);

        return true;
    }

    void CameraLucidArena::acqStop()
    {
        cout << "CameraLucidArena::acqStop"<< endl;

        arv_stream_get_statistics(stream, &nbCompletedBuffers, &nbFailures, &nbUnderruns);

        //cout << "Completed buffers = " << (unsigned long long) nbCompletedBuffers   << endl;
        //cout << "Failures          = " << (unsigned long long) nbFailures           << endl;
        //cout << "Underruns         = " << (unsigned long long) nbUnderruns          << endl;

        BOOST_LOG_SEV(logger, notification) << "Completed buffers = " << (unsigned long long) nbCompletedBuffers;
        BOOST_LOG_SEV(logger, notification) << "Failures          = " << (unsigned long long) nbFailures;
        BOOST_LOG_SEV(logger, notification) << "Underruns         = " << (unsigned long long) nbUnderruns;

        BOOST_LOG_SEV(logger, notification) << "Stopping acquisition...";
        arv_camera_stop_acquisition(camera, &error);
        ErrorManager::CheckAravisError(&error);

        BOOST_LOG_SEV(logger, notification) << "Acquisition stopped.";

        BOOST_LOG_SEV(logger, notification) << "Unreferencing stream.";
        g_object_unref(stream);
        stream = NULL;

    }

    bool CameraLucidArena::grabImage(Frame &newFrame)
    {
        //cout << "CameraLucidArena::grabImage"<< endl;

        ArvBuffer *arv_buffer;
        //exp = arv_camera_get_exposure_time(camera);

        arv_buffer = arv_stream_timeout_pop_buffer(stream,2000000); //us
        char *buffer_data;
        size_t buffer_size;

        if(arv_buffer == NULL){

            throw runtime_error("arv_buffer is NULL");
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

                    Mat image;
                    CamPixFmt imgDepth = MONO8;
                    int saturateVal = 0;

                    if(pixFormat == ARV_PIXEL_FORMAT_MONO_8)
                    {
                        //BOOST_LOG_SEV(logger, normal) << "Creating Mat 8 bits ...";
                        image = Mat(mHeight, mWidth, CV_8UC1, buffer_data);
                        imgDepth = MONO8;
                        saturateVal = 255;

                    }
                    else if(pixFormat == ARV_PIXEL_FORMAT_MONO_12)
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
                    else if(pixFormat == ARV_PIXEL_FORMAT_MONO_16)
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

            }
        }
    }


    bool CameraLucidArena::grabSingleImage(Frame &frame, int camID)
    {
        cout << "CameraLucidArena::grabSingleImage"<< endl;

        bool res = false;

        if(!createDevice(camID))
            return false;

        if(!setPixelFormat(frame.mFormat))
            return false;

        if(!setExposureTime(frame.mExposure))
            return false;

        if(!setGain(frame.mGain))
            return false;

        if(frame.mWidth > 0 && frame.mHeight > 0) {

            setFrameSize(frame.mStartX, frame.mStartY, frame.mWidth, frame.mHeight,1);
            //arv_camera_set_region(camera, frame.mStartX, frame.mStartY, frame.mWidth, frame.mHeight);
            //arv_camera_get_region (camera, NULL, NULL, &mWidth, &mHeight);

        }else{

            int sensor_width, sensor_height;

            arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);
            ErrorManager::CheckAravisError(&error);

            // Use maximum sensor size.
            arv_camera_set_region(camera, 0, 0,sensor_width,sensor_height, &error);
            ErrorManager::CheckAravisError(&error);

            arv_camera_get_region (camera, NULL, NULL, &mWidth, &mHeight, &error);
            ErrorManager::CheckAravisError(&error);

        }

        payload = arv_camera_get_payload (camera, &error);
        ErrorManager::CheckAravisError(&error);

        pixFormat = arv_camera_get_pixel_format (camera, &error);
        ErrorManager::CheckAravisError(&error);

        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax, &error);
        ErrorManager::CheckAravisError(&error);

        arv_camera_get_gain_bounds (camera, &gainMin, &gainMax, &error);
        ErrorManager::CheckAravisError(&error);

        arv_camera_set_frame_rate(camera, frame.mFps, &error); /* Regular captures */
        ErrorManager::CheckAravisError(&error);

        fps = arv_camera_get_frame_rate(camera, &error);
        ErrorManager::CheckAravisError(&error);

        capsString = arv_pixel_format_to_gst_caps_string(pixFormat);

        gain    = arv_camera_get_gain(camera, &error);
        ErrorManager::CheckAravisError(&error);

        exp     = arv_camera_get_exposure_time(camera, &error);
        ErrorManager::CheckAravisError(&error);

        cout << endl;

        cout << "DEVICE SELECTED : " << arv_camera_get_device_id(camera, &error)    << endl;
        ErrorManager::CheckAravisError(&error);

        cout << "DEVICE NAME     : " << arv_camera_get_model_name(camera, &error)   << endl;
        ErrorManager::CheckAravisError(&error);

        cout << "DEVICE VENDOR   : " << arv_camera_get_vendor_name(camera, &error)  << endl;
        ErrorManager::CheckAravisError(&error);

        cout << "PAYLOAD         : " << payload                             << endl;
        cout << "Start X         : " << mStartX                             << endl
             << "Start Y         : " << mStartY                             << endl;
        cout << "Width           : " << mWidth                               << endl
             << "Height          : " << mHeight                              << endl;
        cout << "Exp Range       : [" << exposureMin    << " - " << exposureMax   << "]"  << endl;
        cout << "Exp             : " << exp                                 << endl;
        cout << "Gain Range      : [" << gainMin        << " - " << gainMax       << "]"  << endl;
        cout << "Gain            : " << gain                                << endl;
        cout << "Fps Range      : [" << fpsMin        << " - " << fpsMax       << "]"  << endl;
        cout << "Fps             : " << fps                                 << endl;
        cout << "Type            : " << capsString                         << endl;

        cout << endl;

        if(arv_camera_is_gv_device (camera)) {

            // http://www.baslerweb.com/media/documents/AW00064902000%20Control%20Packet%20Timing%20With%20Delays.pdf
            // https://github.com/GNOME/aravis/blob/06ac777fc6d98783680340f1c3f3ea39d2780974/src/arvcamera.c

            // Configure the inter packet delay to insert between each packet for the current stream
            // channel. This can be used as a crude flow-control mechanism if the application or the network
            // infrastructure cannot keep up with the packets coming from the device.
            arv_camera_gv_set_packet_delay (camera, 4000, &error);
            ErrorManager::CheckAravisError(&error);

            // Specifies the stream packet size, in bytes, to send on the selected channel for a GVSP transmitter
            // or specifies the maximum packet size supported by a GVSP receiver.
            arv_camera_gv_set_packet_size (camera, 1444, &error);
            ErrorManager::CheckAravisError(&error);

        }

        // Create a new stream object. Open stream on Camera.
        stream = arv_camera_create_stream(camera, NULL, NULL, &error);
        ErrorManager::CheckAravisError(&error);

        if(stream != NULL){

            if(ARV_IS_GV_STREAM(stream)){

                bool            arv_option_auto_socket_buffer   = true;
                bool            arv_option_no_packet_resend     = true;
                unsigned int    arv_option_packet_timeout       = 20;
                unsigned int    arv_option_frame_retention      = 100;

                if(arv_option_auto_socket_buffer){

                    g_object_set(stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0, NULL);

                }

                if(arv_option_no_packet_resend){

                    g_object_set(stream, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, NULL);

                }

                g_object_set(stream, "packet-timeout", (unsigned)40000, "frame-retention", (unsigned) 200000,NULL);

            }

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

                    if(pixFormat == ARV_PIXEL_FORMAT_MONO_8){

                        Mat image = Mat(mHeight, mWidth, CV_8UC1, buffer_data);
                        image.copyTo(frame.mImg);

                    }else if(pixFormat == ARV_PIXEL_FORMAT_MONO_12 || pixFormat == ARV_PIXEL_FORMAT_MONO_16) {

                        // Unsigned short image.
                        Mat image = Mat(mHeight, mWidth, CV_16UC1, buffer_data);

                        // http://www.theimagingsource.com/en_US/support/documentation/icimagingcontrol-class/PixelformatY16.htm
                        // Some sensors only support 10-bit or 12-bit pixel data. In this case, the least significant bits are don't-care values.
                        if(shiftBitsImage && pixFormat != ARV_PIXEL_FORMAT_MONO_16 ){
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

    void CameraLucidArena::saveGenicamXml(string p){
        cout << "CameraLucidArena::saveGenicamXml"<< endl;

        const char *xml;

        size_t size;

        xml = arv_device_get_genicam_xml (arv_camera_get_device(camera), &size);

        if (xml != NULL){

            ofstream infFile;
            string infFilePath = p + "genicam.xml";
            infFile.open(infFilePath.c_str());
            infFile << string ( xml, size );
            infFile.close();

        }

    }

    //https://github.com/GNOME/aravis/blob/b808d34691a18e51eee72d8cac6cfa522a945433/src/arvtool.c
    void CameraLucidArena::getAvailablePixelFormats() {
        cout << "CameraLucidArena::getAvailablePixelFormats"<< endl;

        ArvGc *genicam;
        ArvDevice *device;
        ArvGcNode *node;

        if(camera != NULL) {

            device = arv_camera_get_device(camera);
            genicam = arv_device_get_genicam(device);
            node = arv_gc_get_node(genicam, "PixelFormat");

            if (ARV_IS_GC_ENUMERATION (node)) {

                const GSList *childs;
                const GSList *iter;
                vector<string> pixfmt;

                cout << ">> Device pixel formats :" << endl;

                childs = arv_gc_enumeration_get_entries (ARV_GC_ENUMERATION (node));
                for (iter = childs; iter != NULL; iter = iter->next) {
                    if (arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (iter->data), NULL)) {

                        if(arv_gc_feature_node_is_available (ARV_GC_FEATURE_NODE (iter->data), NULL)) {

                            {
                                string fmt = string(arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE (iter->data)));
                                std::transform(fmt.begin(), fmt.end(),fmt.begin(), ::toupper);
                                pixfmt.push_back(fmt);
                                cout << "- " << fmt << endl;

                            }
                        }
                    }
                }

                // Compare found pixel formats to currently formats supported by freeture

                cout << endl <<  ">> Available pixel formats :" << endl;
                EParser<CamPixFmt> fmt;

                for( int i = 0; i != pixfmt.size(); i++ ) {

                    if(fmt.isEnumValue(pixfmt.at(i))) {

                        cout << "- " << pixfmt.at(i) << " available --> ID : " << fmt.parseEnum(pixfmt.at(i)) << endl;

                    }

                }

            }else {

                cout << ">> Available pixel formats not found." << endl;

            }

            g_object_unref(device);

        }

    }


    void CameraLucidArena::getFPSBounds(double &fMin, double &fMax)
    {
        cout << "CameraLucidArena::getFPSBounds ";

        double fpsMin = 0.0;
        double fpsMax = 0.0;

        arv_camera_get_frame_rate_bounds(camera, &fpsMin, &fpsMax, &error);
        ErrorManager::CheckAravisError(&error);

        fMin = fpsMin;
        fMax = fpsMax;

        cout <<"["<< fpsMin<<", "<< fpsMax<<"]"<<endl;
    }



    void CameraLucidArena::getExposureBounds(double &eMin, double &eMax)
    {

        cout << "CameraLucidArena::getExposureBounds ";

        double exposureMin = 0.0;
        double exposureMax = 0.0;

        arv_camera_get_exposure_time_bounds(camera, &exposureMin, &exposureMax, &error);
        ErrorManager::CheckAravisError(&error);

        eMin = exposureMin;
        eMax = exposureMax;
        cout <<"["<< exposureMin<<", "<< exposureMax<<"]"<<endl;
    }

    double CameraLucidArena::getExposureTime()
    {
        cout << "CameraLucidArena::getExposureTime"<< endl;

        double result = arv_camera_get_exposure_time(camera, &error);
        ErrorManager::CheckAravisError(&error);
        return result;
    }

    void CameraLucidArena::getGainBounds(double &gMin, double &gMax){
        cout << "CameraLucidArena::getGainBounds"<< endl;


        double gainMin = 0.0;
        double gainMax = 0.0;

        arv_camera_get_gain_bounds(camera, &gainMin, &gainMax, &error);
        ErrorManager::CheckAravisError(&error);

        gMin = gainMin;
        gMax = gainMax;

    }

    bool CameraLucidArena::getPixelFormat(CamPixFmt &format){
        cout << "CameraLucidArena::getPixelFormat"<< endl;

        ArvPixelFormat pixFormat = arv_camera_get_pixel_format(camera, &error);
        ErrorManager::CheckAravisError(&error);

        switch(pixFormat){

            case ARV_PIXEL_FORMAT_MONO_8 :

                format = MONO8;

                break;

            case ARV_PIXEL_FORMAT_MONO_12 :

                format = MONO12;

                break;
           case ARV_PIXEL_FORMAT_MONO_16 :

                format = MONO16;

                break;

            default :

                return false;

                break;

        }

        return true;
    }


    bool CameraLucidArena::getFrameSize(int &x, int &y, int &w, int &h) {
        cout << "CameraLucidArena::getFrameSize"<< endl;

        if(camera != NULL) {

            int ww = 0, hh = 0, xx = 0, yy = 0;
            arv_camera_get_region(camera, &x, &y, &w, &h,&error);
            ErrorManager::CheckAravisError(&error);
            x = xx;
            y = yy;
            w = ww;
            h = hh;

        }

        return false;

    }

    bool CameraLucidArena::getFPS(double &value){
        cout << "CameraLucidArena::getFPS"<< endl;

        if(camera != NULL) {

            value = arv_camera_get_frame_rate(camera,&error);
            ErrorManager::CheckAravisError(&error);

            return true;

        }

        return false;
    }

    string CameraLucidArena::getModelName(){
        cout << "CameraLucidArena::getModelName"<< endl;

        string result =  arv_camera_get_model_name(camera,&error);
        ErrorManager::CheckAravisError(&error);

        return result;
    }

    bool CameraLucidArena::setExposureTime(double val)
    {
        cout << "CameraLucidArena::setExposureTime ["<< val<<"]" << endl;

        double expMin, expMax;

        arv_camera_get_exposure_time_bounds(camera, &expMin, &expMax,&error);
        ErrorManager::CheckAravisError(&error);

        cout << "arv_camera_get_exposure_time_bounds ["<< expMin<< ", "<< expMax<<"]" << endl;

        if(camera != NULL) {


            if(val >= expMin && val <= expMax) {

                exp = val;
                arv_camera_set_exposure_time(camera, val, &error);
                ErrorManager::CheckAravisError(&error);

                double min, max;
                getFPSBounds(min,max);

            } else {

                cout << "> Exposure value (" << val << ") is not in range [ " << expMin << " - " << expMax << " ]" << endl;
                return false;

            }

            return true;

        }

        return false;
    }

    bool CameraLucidArena::setGain(double val){
        cout << "CameraLucidArena::setGain"<< endl;

        double gMin, gMax;

        arv_camera_get_gain_bounds (camera, &gMin, &gMax, &error);
        ErrorManager::CheckAravisError(&error);

        if (camera != NULL){

            if((double)val >= gMin && (double)val <= gMax){

                gain = val;
                arv_camera_set_gain (camera, (double)val,&error);
                ErrorManager::CheckAravisError(&error);

            }else{

                cout << "> Gain value (" << val << ") is not in range [ " << gMin << " - " << gMax << " ]" << endl;
                BOOST_LOG_SEV(logger, fail) << "> Gain value (" << val << ") is not in range [ " << gMin << " - " << gMax << " ]";
                return false;

            }

        return true;

        }

        return false;

    }

    bool CameraLucidArena::setFPS(double fps)
    {
        cout << "CameraLucidArena::setFPS"<<" ["<<fps<<"]";

        if (camera != NULL)
        {
            //ADDED FOR DEBUG
            double minFps,maxFps;
            getFPSBounds(minFps,maxFps);

            //NEED TO ENABLE FRAME RATE  SETUP
            arv_device_set_boolean_feature_value(arv_camera_get_device (camera), "AcquisitionFrameRateEnable" , true, &error);

            arv_camera_set_frame_rate(camera, fps, &error);
            ErrorManager::CheckAravisError(&error);

            double setfps = arv_camera_get_frame_rate(camera, &error);
            ErrorManager::CheckAravisError(&error);
            cout << "=="<<setfps<<endl;

            int fps_test_l = (int) setfps;
            int fps_test_r = (int) fps;

            if (fps_test_l!=fps_test_r) {
                cout << "> Frame rate value (" << fps << ") can't be set! Please check genicam features." << endl;
                BOOST_LOG_SEV(logger, warning) << "> Frame rate value (" << fps << ") can't be set!";
            }

            return true;

        }

        return false;

    }

    bool CameraLucidArena::setPixelFormat(CamPixFmt depth){
        cout << "CameraLucidArena::setPixelFormat"<< endl;

        if (camera != NULL){

            switch(depth){

                case MONO8 :
                    {
                        arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_8,&error);
                        ErrorManager::CheckAravisError(&error);
                    }
                    break;

                case MONO12 :
                    {
                        arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_12,&error);
                        ErrorManager::CheckAravisError(&error);
                    }
                    break;
                case MONO16 :
                    {
                        arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_16,&error);
                        ErrorManager::CheckAravisError(&error);
                    }
                    break;
            }

            return true;
        }

        return false;

    }

    bool CameraLucidArena::setFrameSize(int startx, int starty, int width, int height, bool customSize) {
        cout << "CameraLucidArena::setFrameSize"<< endl;

        if (camera != NULL){
            if(customSize) {

                if (arv_device_get_feature(arv_camera_get_device(camera), "OffsetX")) {
                    cout << "Starting from : " << mStartX << "," << mStartY;
                    BOOST_LOG_SEV(logger, notification) << "Starting from : " << mStartX << "," << mStartY;
                } else {
                    BOOST_LOG_SEV(logger, warning) << "OffsetX, OffsetY are not available: cannot set offset.";
                }

                arv_camera_set_region(camera, startx, starty, width, height,&error);
                ErrorManager::CheckAravisError(&error);

                arv_camera_get_region (camera, &mStartX, &mStartY, &mWidth, &mHeight,&error);
                ErrorManager::CheckAravisError(&error);

            // Default is maximum size
            } else {

                int sensor_width, sensor_height;

                arv_camera_get_sensor_size(camera, &sensor_width, &sensor_height, &error);
                ErrorManager::CheckAravisError(&error);

                BOOST_LOG_SEV(logger, notification) << "Camera sensor size : " << sensor_width << "x" << sensor_height;

                arv_camera_set_region(camera, 0, 0,sensor_width,sensor_height,&error);
                ErrorManager::CheckAravisError(&error);

                arv_camera_get_region (camera, NULL, NULL, &mWidth, &mHeight,&error);
                ErrorManager::CheckAravisError(&error);

            }
            return true;
        }
        return false;
    }

#endif
