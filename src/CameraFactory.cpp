#include "CameraFactory.h"

#include <memory>

#include "SParam.h"
#include "Logger.h"
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
#include "CameraGigeAravis.h"
#include "CameraLucidAravis.h"
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
        LOG_DEBUG << "CameraFactory::createCamera";
        Camera* camera;
        // shared_ptr<CameraDeviceManager> m_CameraDeviceManager = CameraDeviceManager::Get();
        //     Device* device = m_CameraDeviceManager->getDevice();
        // 
        //     if (device->mCam != nullptr)
        //     {
        //         LOG_ERROR << "CameraFactory::createCamera;" << "MEMORY LEAKING";
        //         assert(device->mCam != nullptr);
        //         free(device->mCam);
        // 
        //         return false;
        //     }
        switch (camera_descriptor.Sdk)
        {
        case CamSdkType::VIDEOFILE:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: VIDEOFILE";
            camera = new CameraVideo(camera_descriptor, runtime_configuration.camInput, runtime_configuration.vidInput.INPUT_VIDEO_PATH, true);
            break;
        }
        case CamSdkType::FRAMESDIR:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: FRAMESDIR";
            // Create camera using pre-recorded fits2D in input.
            camera = new CameraFrames(camera_descriptor, runtime_configuration.camInput, runtime_configuration.framesInput.INPUT_FRAMES_DIRECTORY_PATH, 1, true);
            break;
        }
        case CamSdkType::V4L2:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: V4L2";
#ifdef LINUX
            camera = new CameraV4l2(camera_descriptor, runtime_configuration.camInput);
#endif
            break;
        }
        case CamSdkType::VIDEOINPUT:

        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: VIDEOINPUT";
#ifdef VIDEOINPUT
            camera = new CameraWindows(camera_descriptor, runtime_configuration.camInput);
#endif
            break;
        }
        case CamSdkType::ARAVIS:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: ARAVIS";
#ifdef LINUX
            camera = new CameraGigeAravis(camera_descriptor, runtime_configuration.camInput);
#endif
            break;
        }
        case CamSdkType::LUCID_ARAVIS:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: LUCID_ARAVIS";
#ifdef LINUX
            camera = new CameraGigeAravis(camera_descriptor, runtime_configuration.camInput);
#endif
            break;
        }
        case CamSdkType::LUCID_ARENA:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: LUCID_ARENA";

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
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: PYLONGIGE";
#ifdef USE_PYLON
            camera = new CameraGigePylon(camera_descriptor, runtime_configuration.camInput);
#endif
            break;

        }
        case CamSdkType::TIS:
        {
            LOG_DEBUG << "CameraFactory::createCamera;" << "SDK: TIS";
#ifdef TISCAMERA
            camera = new CameraGigeTis(camera_descriptor, runtime_configuration.camInput);
#endif
            break;
        }

        }

        if (camera == nullptr)
        {
            LOG_ERROR << "CameraFactory::createCamera;" << "Failed to create camera";
            return nullptr;
        }


        if (!camera->initSDK())
        {
            LOG_ERROR << "CameraFactory::createCamera;" << "Failed to init SDK";
            delete camera;
            return nullptr;
        }
        else {

            if (!camera->createDevice()) {
                LOG_ERROR << "CameraFactory::createCamera;" << "Failed to create device";
                delete camera;
                return nullptr;
            }
            else {
                if (!camera->initOnce())
                {
                    LOG_ERROR << "CameraFactory::createCamera;" << "Failed to init once the device";
                    camera->reset();
                    delete camera;
                    return nullptr;
                }
                else if (!camera->init()) {
                    LOG_ERROR << "CameraFactory::createCamera;" << "Failed to init device";
                    camera->reset();
                    delete camera;
                    return nullptr;
                }
                else if (!camera->grabInitialization())
                {
                    LOG_ERROR << "CameraFactory::createCamera;" << "Failed the grab initialization";
                    camera->reset();
                    delete camera;
                    return nullptr;
                }

                //camera->fetchBounds(runtime_configuration);

                if (camera->configurationCheck(runtime_configuration))
                    camera->configure(runtime_configuration);
                else
                    LOG_WARNING << "CameraFactory::createCamera;" << "The configuration failed - the camera may be operate inconsistently";

                return camera;
            }
        }
    }
    catch (...) 
    {
        LOG_ERROR << "CameraFactory::createCamera;" << "Error creting camera see log for full description";
    }

    return nullptr;
}


