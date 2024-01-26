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

#pragma once

#include <vector>

#include "EInputDeviceType.h"
#include "CfgParam.h"
#include "CfgLoader.h"
#include "Device.h"
#include "CameraDeviceManager.h"
#include "CameraFirstInit.h"

namespace freeture {

    class CfgParam{

        private :
            
            static class Init {

                public :

                    Init() {

                        m_Logger.add_attribute("ClassName", boost::log::attributes::constant<std::string>("CfgParam"));

                    }

            }initializer;
            std::string m_CfgFilePath;

            static boost::log::sources::severity_logger< LogSeverityLevel > m_Logger;
            std::vector<std::string> m_EMsg;

            CfgLoader m_Cfg;
            Device* mDevice;
            CameraDeviceManager& manager = CameraDeviceManager::Get();
            parameters m_Param;
            InputDeviceType m_InputType;

            void loadDeviceID();
            void loadDeviceID_old();
            void loadDataParam();
            void loadLogParam();
            void loadFramesParam();
            void loadVidParam();
            void loadCamParam();
            void loadDetParam();
            void loadStackParam();
            void loadStationParam();
            void loadFitskeysParam();
            void loadMailParam();
            void loadCameraSerial();
            void loadCameraInit();


        public :

            bool showErrors;

            /**
             * Constructor.
             *
             */
            CfgParam(Device*,std::string);

            int             getDeviceID();
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
            

            bool deviceIdIsCorrect();
            bool dataParamIsCorrect();
            bool logParamIsCorrect();
            bool framesParamIsCorrect();
            bool vidParamIsCorrect();
            bool camParamIsCorrect();
            bool detParamIsCorrect();
            bool stackParamIsCorrect();
            bool stationParamIsCorrect();
            bool fitskeysParamIsCorrect();
            bool mailParamIsCorrect();
            bool allParamAreCorrect();
            bool inputIsCorrect();

            void setInitRequired(bool);

    };
}
