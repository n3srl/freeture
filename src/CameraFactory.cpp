#include "CameraFactory.h"

#include "Logger.h"

#include <memory>

#include "SParam.h"
#include "CameraVideo.h"
#include "CameraFrames.h"
#include "CameraScanner.h"
#include "ECamSdkType.h"
#include "ECamModel.h"

#ifdef USE_ARENA
#include "CameraLucidArena_PHX016S.h"
#endif

#ifdef LINUX
#include "CameraV4l2.h"
#ifdef USE_ARAVIS
#include "CameraGigeAravis.h"
#include "CameraLucidAravis.h"
#endif
#endif

#ifdef VIDEOINPUT
#include "CameraWindows.h"
#endif

#ifdef TISCAMERA
#include "CameraGigeTis.h"
#endif

#ifdef USE_PYLON
#include "CameraGigePylon.h"
#endif

using namespace freeture;
using namespace std;

Camera* CameraFactory::createCamera(CameraDescription camera_descriptor, parameters& runtime_configuration)
{
    try {
        LOG_DEBUG << "CameraFactory::createCamera" << endl;
        Camera* camera;
        
        switch (camera_descriptor.Sdk)
        {
        case CamSdkType::VIDEOFILE:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: VIDEOFILE" << endl;
            camera = new CameraVideo(camera_descriptor, runtime_configuration.camInput, runtime_configuration.vidInput.INPUT_VIDEO_PATH, true);
            break;
        }
        case CamSdkType::FRAMESDIR:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: FRAMESDIR" << endl;
            // Create camera using pre-recorded fits2D in input.
            camera = new CameraFrames(camera_descriptor, runtime_configuration.camInput, runtime_configuration.framesInput.INPUT_FRAMES_DIRECTORY_PATH, 1, true);
            break;
        }
        case CamSdkType::V4L2:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: V4L2" << endl;
#ifdef LINUX
            camera = new CameraV4l2(camera_descriptor, runtime_configuration.camInput);
#endif
            break;
        }
        case CamSdkType::VIDEOINPUT:

        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: VIDEOINPUT" << endl;
#ifdef VIDEOINPUT
            camera = new CameraWindows(camera_descriptor, runtime_configuration.camInput);
#endif
            break;
        }
        case CamSdkType::ARAVIS:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: ARAVIS" << endl;
#ifdef LINUX
#ifdef USE_ARAVIS
            camera = new CameraGigeAravis(camera_descriptor, runtime_configuration.camInput);
#endif
#endif
            break;
        }
        case CamSdkType::LUCID_ARAVIS:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: LUCID_ARAVIS" << endl;
#ifdef LINUX
#ifdef USE_ARAVIS
            camera = new CameraLucidAravis(camera_descriptor, runtime_configuration.camInput);
#endif
#endif
            break;
        }
        case CamSdkType::LUCID_ARENA:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: LUCID_ARENA" << endl;

#ifdef USE_ARENA
            switch (camera_descriptor.Model) {
            case CamModel::NONE:
            case CamModel::LUCID_PHOENIX_016S_MS:
            case CamModel::LUCID_PHOENIX_032S_MS:
                camera = new CameraLucidArena_PHX016S(camera_descriptor, runtime_configuration.camInput);
            }
#endif
            break;
        }
        case CamSdkType::PYLONGIGE:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: PYLONGIGE" << endl;
#ifdef USE_PYLON
            camera = new CameraGigePylon(camera_descriptor, runtime_configuration.camInput);
#endif
            break;

        }
        case CamSdkType::TIS:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: TIS" << endl;
#ifdef TISCAMERA
            camera = new CameraGigeTis(camera_descriptor, runtime_configuration.camInput);
#endif
            break;
        }

        }

        if (camera == nullptr)
        {
            LOG_ERROR << "CameraFactory::createCamera;" << "Failed to create camera" << endl;
            return nullptr;
        }


        if (!camera->initSDK())
        {
            LOG_ERROR << "CameraFactory::createCamera;" << "Failed to init SDK" << endl;
            delete camera;
            return nullptr;
        }
        else {

            if (!camera->createDevice()) {
                LOG_ERROR << "CameraFactory::createCamera;" << "Failed to create device" << endl;
                delete camera;
                return nullptr;
            }
            else {
                if (!camera->initOnce())
                {
                    LOG_ERROR << "CameraFactory::createCamera;" << "Failed to init once the device" << endl;
                    camera->reset();
                    delete camera;
                    return nullptr;
                }
                else if (!camera->init()) {
                    LOG_ERROR << "CameraFactory::createCamera;" << "Failed to init device" << endl;
                    camera->reset();
                    delete camera;
                    return nullptr;
                }
                else if (!camera->grabInitialization())
                {
                    LOG_ERROR << "CameraFactory::createCamera;" << "Failed the grab initialization" << endl;
                    camera->reset();
                    delete camera;
                    return nullptr;
                }

                //camera->fetchBounds(runtime_configuration);

                if (camera->configurationCheck(runtime_configuration))
                    camera->configure(runtime_configuration);
                else
                    LOG_WARNING << "CameraFactory::createCamera;" << "The configuration failed - the camera may be operate inconsistently" << endl;

                return camera;
            }
        }
    }
    catch (...) 
    {
        LOG_ERROR << "CameraFactory::createCamera;" << "Error creting camera see log for full description" << endl;
    }

    return nullptr;
}


