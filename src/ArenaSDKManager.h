#pragma once
#include "Commons.h"

#include <ArenaApi.h>

#include <memory>
#include <string>
#include <mutex>

#define EXPLORE_DEVICE true
#define EXPLORE_TL_DEVICE true
#define EXPLORE_TL_STREAM true
#define EXPLORE_TL_INTERFACE true
#define EXPLORE_TL_SYSTEM true


// Choose node properties to explore
#define EXPLORE_ACCESS true
#define EXPLORE_VISIBILITY true
#define EXPLORE_TYPE true
#define EXPLORE_VALUE true


#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "      "

namespace freeture
{
    enum class ArenaSwitch {
        ON,
        OFF
    };

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
        static bool setBooleanValue(Arena::IDevice*, std::string, bool);
        static bool setSwitchValue(Arena::IDevice*, std::string, ArenaSwitch);
        static bool setFloatValue(Arena::IDevice*, std::string, double);
        static bool setStringValue(Arena::IDevice*, std::string, std::string);
        static bool sendCommand(Arena::IDevice*, std::string);


        /// <summary>
        /// explores a node map
        /// (1) retrieves total number of nodes
        /// (2) retrieves category nodes
        /// </summary>
        /// <param name=""></param>
        static void exploreNodeMap(GenApi::INodeMap* );
        /// <summary>
        /// explores node maps
        /// (1) retrieves node map from device
        /// (2) retrieves node maps from corresponding transport layer modules
        /// (3) explores the device node map
        /// (4) explores the transport layer device node map
        /// (5) explores the transport layer stream node map
        /// (6) explores the transport layer interface node map
        /// (7) explores the transport layer system node map
        /// </summary>
        /// <param name=""></param>
        /// <param name=""></param>
        static void exploreNodeMaps(Arena::IDevice*);

        /// <summary>
        /// explores node
        /// (1) retrieves display name
        /// (2) retrieves node name
        /// (3) retrieves accessibility
        /// (4) retrieves visibility
        /// (5) retrieves interface type
        /// (6) retrieves value
        /// </summary>
        static void exploreNode(GenApi::INode* pNode, size_t depth = 0);

        /// <summary>
        /// explores nodes of boolean type
        /// (1) retrieves value
        /// (2) demonstrates value setter in a comment
        /// </summary>
        /// <param name="pBoolean"></param>
        static void exploreBoolean(GenApi::CBooleanPtr pBoolean);
        /// <summary>
        /// explores nodes of type float
        /// (1) retrieves value
        /// (2) retrieves minimum and maximum
        /// (3) retrieves increment and increment mode
        /// (4) retrieves representation
        /// (5) retrieves unit
        /// (6) retrieves display notation
        /// (7) retrieves display precision
        /// (8) demonstrates maximum and minimum imposition in a comment
        /// (9) demonstrates value setter in a comment
        /// </summary>
        static void exploreFloat(GenApi::CFloatPtr pFloat);

        /// <summary>
        /// explores nodes of type integer
        /// (1) retrieves value
        /// (2) retrieves maximum and minimum
        /// (3) retrieves increment and increment mode
        /// (4) retrieves representation
        /// (5) retrieves unit
        /// (6) demonstrates maximum and minimum imposition in a comment
        /// (7) demonstrates value setter in a comment
        /// </summary>
        static void exploreInteger(GenApi::CIntegerPtr pInteger);

        /// <summary>
        /// explores nodes of type enumeration
        /// (1) retrieves currently set enum entry node
        /// (2) retrieves value
        /// (3) retrieves list of entries
        /// (4) demonstrates list of symbolics retrieval in a comment
        /// (5) demonstrates enum entry node retrieval by name in a comment
        /// (6) demonstrates value setter in a comment
        /// </summary>
        static void exploreEnumeration(GenApi::CEnumerationPtr pEnumeration);

        /// <summary>
        /// explores nodes of string type
        /// (1) retrieves value
        /// (2) retrieves maximum value length
        /// (3) demonstrates value setter in a comment
        /// </summary>
        static void exploreString(GenApi::CStringPtr pString);
    };

}