#include "CameraScanner.h"

#include "Logger.h"

#include <string>

#include "CameraLucidArena_Scanner.h"

#ifdef LINUX

#include "CameraGigeAravis_Scanner.h"
#include "CameraLucidAravis_Scanner.h"

#endif

using namespace std;
using namespace freeture;



CameraScanner::CameraScanner(CamSdkType sdk) :Sdk(sdk)
{
}

CameraScanner::~CameraScanner() {

}


/**
  Update the List of camera descriptions and return back the vector
 */
vector<CameraDescription>& CameraScanner::getCamerasList()
{
    UpdateCameraList();
    return Devices;
}

/**
  * Scanner factory method
  */
CameraScanner* CameraScanner::CreateScanner(CamSdkType sdk)
{

    switch (sdk)
    {
#ifdef LINUX
        case CamSdkType::ARAVIS:
            return new CameraGigeAravis_Scanner(sdk);
        case CamSdkType::LUCID_ARAVIS:
            return new CameraLucidAravis_Scanner(sdk);
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
    LOG_DEBUG << "CameraLucidArena::listCameras";
    LOG_DEBUG << "------------ CAMERAS ----------";

    for(int i = 0; i < Devices.size(); i++)
        LOG_DEBUG << "-> [" << Devices[i].Id << "] " << Devices[i].Description;

    LOG_DEBUG << "-------------------------------";

    return true;
}
