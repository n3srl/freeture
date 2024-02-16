#include "CameraScanner.h"

#include <string>

#include "CameraGigeAravisScanner.h"
#include "CameraLucidArenaScanner.h"
#include "CameraLucidArena_Scanner.h"

#include "Logger.h"

using namespace std;
using namespace freeture;

/**
  Update the List of camera descriptions and return back the vector
 */
vector<CameraDescription>& CameraScanner::getCamerasList()
{
    UpdateCameraList();
    return Devices;
}

/**
  * Scanneer factory method
  */
CameraScanner* CameraScanner::CreateScanner(CamSdkType sdk)
{

    switch (sdk)
    {
#ifdef LINUX
        case CamSdkType::ARAVIS:
            return new CameraGigeAravisScanner(sdk);
        case CamSdkType::LUCID_ARAVIS:
            return new CameraLucidArenaScanner(sdk);
#endif
        case CamSdkType::LUCID_ARENA:
            return new CameraLucidArena_Scanner(sdk);

        case CamSdkType::FRAMESDIR:
        case CamSdkType::PYLONGIGE:
        case CamSdkType::TIS:
        case CamSdkType::UNKNOWN:
        case CamSdkType::V4L2:
        case CamSdkType::VIDEOFILE:
        case CamSdkType::VIDEOINPUT:
        return nullptr;
    };

    return nullptr;
}

/**
  * Print a list with connected devices.
  *
  */
bool CameraScanner::listCameras()
{
    LOG_DEBUG << "CameraLucidArena::listCameras"<< endl;
    LOG_DEBUG << endl << "------------ GIGE CAMERAS WITH ARAVIS ----------" << endl << endl;

    for(int i = 0; i < Devices.size(); i++)
        LOG_DEBUG << "-> [" << Devices[i].Id << "] " << Devices[i].Description<< endl;

    LOG_DEBUG << endl << "------------------------------------------------" << endl << endl;

    return true;
}
