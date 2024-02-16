#pragma once
#include "Commons.h"

#include <memory>
#include <mutex>


namespace Arena {
    class ISystem;
}

namespace freeture
{

    class ArenaSDKManager
    {
    private:

        static Arena::ISystem* m_ArenaSDKSystem;
        static std::shared_ptr<ArenaSDKManager> m_Instance;
        static std::mutex m_Mutex;

    public:
        ArenaSDKManager();
        ~ArenaSDKManager();

        static std::shared_ptr<Arena::ISystem> Get();
    };

}