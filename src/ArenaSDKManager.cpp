#include "ArenaSDKManager.h"
#include <ArenaApi.h>

using namespace freeture;

std::shared_ptr<ArenaSDKManager> ArenaSDKManager::m_Instance;

std::mutex ArenaSDKManager::m_Mutex;
Arena::ISystem* ArenaSDKManager::m_ArenaSDKSystem = nullptr;

ArenaSDKManager::ArenaSDKManager()
{
    if (m_ArenaSDKSystem == nullptr)
        m_ArenaSDKSystem = Arena::OpenSystem();
}

ArenaSDKManager::~ArenaSDKManager()
{
    if (m_ArenaSDKSystem != nullptr)
        Arena::CloseSystem(m_ArenaSDKSystem);
     
    //m_ArenaSDKSystem = nullptr;
}

std::shared_ptr<Arena::ISystem> ArenaSDKManager::Get()
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    if (!m_Instance)
        m_Instance = std::make_shared<ArenaSDKManager>();

    return std::shared_ptr<Arena::ISystem>(m_ArenaSDKSystem);
}