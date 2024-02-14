#pragma once
/*
                                CfgParam.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
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
*   Last modified:      20/10/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    CfgParam.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
* \brief   FreeTure parameters
*/
#include "Commons.h"

#include <memory>
#include <vector>
#include <string>

#include "EInputDeviceType.h"
#include "CfgLoader.h"
#include "SParam.h"

namespace freeture {

    class CfgParam{

        private :
            /// <summary>
            /// Configuration file path
            /// </summary>
            std::string m_CfgFilePath;

            /// <summary>
            /// Error messages
            /// </summary>
            std::vector<std::string> m_EMsg;
            
            /// <summary>
            /// Configuration loader implementation
            /// </summary>
            CfgLoader m_Cfg;

            /// <summary>
            /// Runtime Freeture parameters
            /// </summary>
            parameters m_Param;

            /// <summary>
            /// Loads CAMERA_ID from configuration
            /// </summary>
            void loadDeviceID();

            /// <summary>
            /// loads DATA_PATH and FITS_COMPRESSION from configuration
            /// </summary>
            void loadDataParam();

            /// <summary>
            /// loads LOG_PATH, LOG_ARCHIVE_DAY, LOG_SIZE_LIMIT, LOG_SEVERITY from configuration
            /// </summary>
            void loadLogParam();

            /// <summary>
            /// loads INPUT_TIME_INTERVAL, INPUT_FRAMES_DIRECTORY_PATH from configuration
            /// </summary>
            void loadFramesParam();

            /// <summary>
            /// loads INPUT_TIME_INTERVAL, INPUT_VIDEO_PATH from configuration
            /// </summary>
            void loadVidParam();

            /// <summary>
            /// loads 
            ///     - DET_ENABLED
            ///     - ACQ_BUFFER_SIZE
            ///     - ACQ_MASK_ENABLED
            ///     if true:
            ///         - ACQ_MASK_PATH
            ///     - DET_MODE 
            ///     - DET_DEBUG
            ///     if true:
            ///         - DET_DEBUG_PATH
            ///     - DET_TIME_AROUND
            ///     - DET_TIME_MAX
            ///     - DET_METHOD
            ///     - DET_SAVE_FITS3D
            ///     - DET_SAVE_FITS2D
            ///     - DET_SAVE_SUM
            ///     - DET_SUM_REDUCTION
            ///     - DET_SUM_MTHD
            ///     - DET_SAVE_SUM_WITH_HIST_EQUALIZATION
            ///     - DET_SAVE_AVI
            ///     - DET_UPDATE_MASK
            ///     - DET_UPDATE_MASK_FREQUENCY
            ///     - DET_DEBUG_UPDATE_MASK
            ///     - DET_DEBUG_PATH
            ///     - DET_DOWNSAMPLE_ENABLED
            ///     - DET_SAVE_GEMAP
            ///     - DET_SAVE_DIRMAP
            ///     - DET_SAVE_POS
            ///     - DET_LE_MAX
            ///     - DET_GE_MAX
            /// from configuration file
            /// </summary>
            void loadDetParam();

            /// <summary>
            /// 
            /// </summary>
            /// <param name=""></param>
            void loadDetParam(InputDeviceType);

            /// <summary>
            /// loads 
            ///     - CAMERA_SERIAL
            ///     - CAMERA_INIT
            ///     - CAMERA_INIT_CONFIG
            ///     - ACQ_FPS
            ///     - ACQ_FORMAT
            ///     - ACQ_RES_CUSTOM_SIZE
            ///     if true:
            ///         - ACQ_OFFSET
            ///         - ACQ_RES_SIZE
            ///     - SHIFT_BITS
            ///     - ACQ_NIGHT_EXPOSURE
            ///     - ACQ_NIGHT_GAIN
            ///     - ACQ_DAY_EXPOSURE
            ///     - ACQ_DAY_GAIN
            ///     - EXPOSURE_CONTROL_ENABLED
            ///     if true:
            ///         - EXPOSURE_CONTROL_FREQUENCY
            ///         - EXPOSURE_CONTROL_SAVE_IMAGE
            ///         - EXPOSURE_CONTROL_SAVE_INFOS
            ///     - EPHEMERIS_ENABLED
            ///     if true:
            ///         - SUNRISE_TIME
            ///         - SUNSET_TIME
            ///         - SUNSET_DURATION
            ///         - SUNRISE_DURATION
            ///     - SUN_HORIZON_1
            ///     - SUN_HORIZON_2
            ///     - ACQ_REGULAR_ENABLED
            ///     if true:
            ///         - ACQ_REGULAR_MODE
            ///         - ACQ_REGULAR_PRFX
            ///         - ACQ_REGULAR_OUTPUT
            ///         - ACQ_REGULAR_CFG
            ///     - ACQ_SCHEDULE_ENABLED
            ///     if true:
            ///         - ACQ_SCHEDULE_OUTPUT
            ///         - ACQ_SCHEDULE
            /// from configuration file
            /// </summary>
            void loadCamParam();

            /// <summary>
            /// loads 
            ///     - STACK_ENABLED
            ///         - STACK_MODE
            ///         - STACK_TIME
            ///         - STACK_INTERVAL
            ///         - STACK_MTHD
            ///         - STACK_REDUCTION
            /// from configuration file
            /// </summary>
            void loadStackParam();

            /// <summary>
            /// loads
            ///     - STATION_NAME
            ///     - TELESCOP
            ///     - OBSERVER
            ///     - INSTRUMENT
            ///     - CAMERA
            ///     - FOCAL
            ///     - APERTURE
            ///     - SITELONG
            ///     - SITELAT
            ///     - SITEELEV
            /// from configuration file
            /// </summary>
            void loadStationParam();

            /// <summary>
            /// loads 
            ///     - K1
            ///     - K2
            ///     - FILTER
            ///     - CD1_1
            ///     - CD1_2
            ///     - CD2_1
            ///     - CD2_2
            ///     - XPIXEL
            ///     - YPIXEL
            ///     - COMMENT
            /// from configuration file
            /// </summary>
            void loadFitskeysParam();
            
            /// <summary>
            /// loads 
            ///     - MAIL_DETECTION_ENABLED
            ///         - MAIL_RECIPIENT
            ///         - MAIL_SMTP_SERVER
            ///         - MAIL_CONNECTION_TYPE
            ///         - MAIL_SMTP_LOGIN
            ///         - MAIL_SMTP_PASSWORD
            /// from configuration file
            /// </summary>
            void loadMailParam();

            /// <summary>
            /// loads CAMERA_SERIAL from configuration file
            /// </summary>
            void loadCameraSerial();

            /// <summary>
            /// loads  CAMERA_INIT and CAMERA_INIT_CONFIG  from configuration file
            /// if CAMERA_INIT is true 
            /// </summary>
            bool loadCameraInit();

            void loadInputParam(InputDeviceType);

        public :
            /// <summary>
            /// Set to true in order to populate error structures
            /// </summary>
            bool enableErrors;

            /// <summary>
            /// default Ctor
            /// </summary>
            /// <param name="cfgFilePath">configuration file path</param>
            CfgParam(std::string);

            dataParam       getDataParam();
            logParam        getLogParam();
            framesParam     getFramesParam();
            videoParam      getVidParam();
            cameraParam     getCamParam();
            detectionParam  getDetParam();
            stackParam      getStackParam();
            stationParam    getStationParam();
            fitskeysParam   getFitskeysParam();
            mailParam       getMailParam();
            parameters      getAllParam();
            
            /// <summary>
            /// Check if device id parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkDeviceID();
            /// <summary>
            /// Check if log parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkLogParam();

            /// <summary>
            /// Check if data parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkDataParam();
            /// <summary>
            /// Check if frames parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkFramesParam();

            /// <summary>
            /// Check if video parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkVidParam();

            /// <summary>
            /// Check if camera parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkCamParam();

            /// <summary>
            /// Check if detection parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkDetParam();

            /// <summary>
            /// Check if stack parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkStackParam();

            /// <summary>
            /// Check if stack parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkStationParam();

            /// <summary>
            /// Check if fitskey parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkFitskeysParam();

            /// <summary>
            /// Check if mail parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool checkMailParam();

            /// <summary>
            /// Check is all parameters loaded correctly
            /// </summary>
            /// <returns>true if correct, false otherwise</returns>
            bool allParamAreCorrect();

            /// <summary>
            /// Test if input parameter are correct applied to specific device type
            /// </summary>
            /// <param name=""></param>
            /// <returns></returns>
            bool checkInputParam(InputDeviceType);


            void setInitRequired(bool);

    };
}
