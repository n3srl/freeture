#include "CameraScanner.h"

#include "CameraGigeAravisScanner.h"
#include "CameraLucidArenaScanner.h"
#include "CameraLucidArena_PHX016SScanner.h"

using namespace std;



/**
  Update the List of camera descriptions and return back the vector
 */
std::vector<CameraDescription>& CameraScanner::getCamerasList()
{
    UpdateCameraList();
    return Devices;
}

/**
  * Scanneer factory
  */
CameraScanner* CameraScanner::CreateScanner(CamSdkType sdk)
{

    switch (sdk)
    {
        case CamSdkType::ARAVIS:
            return new CameraGigeAravisScanner(sdk);

        case CamSdkType::LUCID_ARAVIS:
            return new CameraLucidArenaScanner(sdk);

        case CamSdkType::LUCID_ARENA:
            return new CameraLucidArena_PHX016SScanner(sdk);

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
    cout << "CameraLucidArena::listCameras"<< endl;
    cout << endl << "------------ GIGE CAMERAS WITH ARAVIS ----------" << endl << endl;

    for(int i = 0; i < Devices.size(); i++)
        cout << "-> [" << Devices[i].Id << "] " << Devices[i].Description<< endl;

    cout << endl << "------------------------------------------------" << endl << endl;

    return true;
}
