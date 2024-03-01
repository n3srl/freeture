#include "ArenaSDKManager.h"
#include <iomanip>

using namespace freeture;
using namespace std;

std::shared_ptr<ArenaSDKManager> ArenaSDKManager::m_Instance;

std::mutex ArenaSDKManager::m_Mutex;
Arena::ISystem* ArenaSDKManager::m_ArenaSDKSystem = nullptr;

ArenaSDKManager::ArenaSDKManager()
{
    if (m_ArenaSDKSystem == nullptr) {
        LOG_DEBUG << "ArenaSDKManager::ArenaSDKManager;" << "Allocating Arena SDK System";
        m_ArenaSDKSystem = Arena::OpenSystem();
    }
}

ArenaSDKManager::~ArenaSDKManager()
{
    if (m_ArenaSDKSystem != nullptr) {
        LOG_DEBUG << "ArenaSDKManager::ArenaSDKManager;" << "Deallocating Arena SDK System";
        Arena::CloseSystem(m_ArenaSDKSystem);
    }
}

std::shared_ptr<Arena::ISystem> ArenaSDKManager::Get()
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    if (!m_Instance)
        m_Instance = std::make_shared<ArenaSDKManager>();

    return std::shared_ptr<Arena::ISystem>(m_ArenaSDKSystem);
}

bool ArenaSDKManager::setBooleanValue(Arena::IDevice* device, string feature, bool value)
{
    try {
        Arena::SetNodeValue<bool>(device->GetNodeMap(), feature.c_str(), value);
        return true;
    }
    catch (GenICam::GenericException& ge)
    {
        throw runtime_error(ge.what());
    }
    catch (std::exception& ex)
    {
        throw runtime_error(ex.what());
    }
    catch (...)
    {
        throw runtime_error("Unexpected exception thrown");
    }

}


bool ArenaSDKManager::setIntegerValue(Arena::IDevice* device, string feature, int value)
{
    try {
        Arena::SetNodeValue<int64_t>(device->GetNodeMap(), feature.c_str(), value);
        return true;
    }
    catch (GenICam::GenericException& ge)
    {
        throw runtime_error(ge.what());
    }
    catch (std::exception& ex)
    {
        throw runtime_error(ex.what());
    }
    catch (...)
    {
        throw runtime_error("Unexpected exception thrown");
    }

}

bool ArenaSDKManager::setSwitchValue(Arena::IDevice* device, string feature, ArenaSwitch value)
{
    try {
        switch (value) {
        case ArenaSwitch::ON:
            return setStringValue(device, feature, "On");
        case ArenaSwitch::OFF:
            return setStringValue(device, feature, "Off");
        }
    }
    catch (GenICam::GenericException& ge)
    {
        throw runtime_error(ge.what());
    }
    catch (std::exception& ex)
    {
        throw runtime_error(ex.what());
    }
    catch (...)
    {
        throw runtime_error("Unexpected exception thrown");
    }

    return false;
}

bool ArenaSDKManager::setFloatValue(Arena::IDevice* device, string feature, double value)
{
    try {
        Arena::SetNodeValue<double>(device->GetNodeMap(), feature.c_str(), value);
        return true;
    }
    catch (GenICam::GenericException& ge)
    {
        throw runtime_error(ge.what());
    }
    catch (std::exception& ex)
    {
        throw runtime_error(ex.what());
    }
    catch (...)
    {
        throw runtime_error("Unexpected exception thrown");
    }

    return false;
}

bool ArenaSDKManager::setStringValue(Arena::IDevice* device, string feature, string value)
{
    try {
        Arena::SetNodeValue<GenICam::gcstring>(device->GetNodeMap(), feature.c_str(), value.c_str());
        return true;
    }
    catch (GenICam::GenericException& ge)
    {
        throw runtime_error(ge.what());
    }
    catch (std::exception& ex)
    {
        throw runtime_error(ex.what());
    }
    catch (...)
    {
        throw runtime_error("Unexpected exception thrown");
    }

    return false;
}

bool ArenaSDKManager::sendCommand(Arena::IDevice* device, string command)
{
    try {
        Arena::ExecuteNode(device->GetNodeMap(), command.c_str());
        return true;
    }
    catch (GenICam::GenericException& ge)
    {
        throw runtime_error( ge.what() );
    }
    catch (std::exception& ex)
    {
        throw runtime_error(ex.what());
    }
    catch (...)
    {
        throw runtime_error("Unexpected exception thrown");
    }

    return false;
}

#if ARENA_TOOLS
void ArenaSDKManager::exploreNodeMap(GenApi::INodeMap* pNodeMap)
{
    // Get number of nodes
   LOG_DEBUG << "ArenaSDKManager::ExploreNodeMap;" << TAB2 << "Number of nodes: ";

    uint64_t numNodes = pNodeMap->GetNumNodes();

    LOG_DEBUG << "ArenaSDKManager::ExploreNodeMap;" << numNodes << "\n";

    // Get nodes
    GenApi::NodeList_t nodes;

    pNodeMap->GetNodes(nodes);

    // print category nodes
    LOG_DEBUG <<  "ArenaSDKManager::ExploreNodeMap;" << TAB2 << "Category nodes: ";
    bool firstPass = true;

    for (GenApi::CCategoryPtr pCategoryNode : nodes)
    {
        if (pCategoryNode)
        {
            if (firstPass)
            {
                LOG_DEBUG << "ArenaSDKManager::ExploreNodeMap;" << TAB2 << pCategoryNode->GetNode()->GetDisplayName();
                exploreNode(pCategoryNode->GetNode());
                firstPass = false;
            }
            else
                LOG_DEBUG << "ArenaSDKManager::ExploreNodeMap;" << TAB2 << ", " << pCategoryNode->GetNode()->GetDisplayName();
        }
    }

}

void ArenaSDKManager::exploreNodeMaps(Arena::IDevice* pDevice)
{
    std::cout << "ArenaSDKManager::exploreNodeMaps;" << "Retrieve node maps\n";

    // Retrieve node map from device
    GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();

    // Retrieve node maps from corresponding transport layer modules
    GenApi::INodeMap* pTLDeviceNodeMap = pDevice->GetTLDeviceNodeMap();
    GenApi::INodeMap* pTLStreamNodeMap = pDevice->GetTLStreamNodeMap();
    GenApi::INodeMap* pTLInterfaceNodeMap = pDevice->GetTLInterfaceNodeMap();
    GenApi::INodeMap* pTLSystemNodeMap = m_ArenaSDKSystem->GetTLSystemNodeMap();

    // Explore device node map
    if (EXPLORE_DEVICE)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreNodeMaps;" << TAB1 << "Explore device node map\n";

        exploreNodeMap(pNodeMap);
    }

    // Explore transport layer device node map
    if (EXPLORE_TL_DEVICE)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreNodeMaps;" << TAB1 << "Explore transport layer device node map\n";

        exploreNodeMap(pTLDeviceNodeMap);
    }

    // Explore transport layer stream node map
    if (EXPLORE_TL_STREAM)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreNodeMaps;" << TAB1 << "Explore transport layer stream node map\n";

        exploreNodeMap(pTLStreamNodeMap);
    }

    // Explore transport layer interface node map
    if (EXPLORE_TL_INTERFACE)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreNodeMaps;" << TAB1 << "Explore transport layer interface node map\n";

        exploreNodeMap(pTLInterfaceNodeMap);
    }

    // Explore transport layer system node map
    if (EXPLORE_TL_SYSTEM)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreNodeMaps;" << TAB1 << "Explore transport layer system node map\n";

        exploreNodeMap(pTLSystemNodeMap);
    }
}


std::string Indent(size_t depth)
{
    std::string indentation = "  ";
    for (size_t i = 0; i < depth; i++)
        indentation += "  ";
    return indentation;
}

// explores node
// (1) retrieves display name
// (2) retrieves node name
// (3) retrieves accessibility
// (4) retrieves visibility
// (5) retrieves interface type
// (6) retrieves value
void ArenaSDKManager::exploreNode(GenApi::INode* pNode, size_t depth)
{
    // Retrieve display name
    GenICam::gcstring displayName = pNode->GetDisplayName();

    // Retrieve node name
    GenICam::gcstring nodeName = pNode->GetName();

    // Retrieve accessibility
    GenApi::EAccessMode accessMode = pNode->GetAccessMode();
    GenICam::gcstring accessModeStr = GenApi::EAccessModeClass::ToString(accessMode);

    // Retrieve visibility
    GenApi::EVisibility visibility = pNode->GetVisibility();
    GenICam::gcstring visibilityStr = GenApi::EVisibilityClass::ToString(visibility);

    // Retrieve interface type
    GenApi::EInterfaceType interfaceType = pNode->GetPrincipalInterfaceType();
    GenICam::gcstring interfaceTypeStr = Arena::EInterfaceTypeClass::ToString(interfaceType);

    // Retrieve value
    GenICam::gcstring value = "-";
    if (GenApi::IsReadable(pNode))
    {
        GenApi::CValuePtr pValue = pNode;
        value = pValue->ToString();
    }

    if (pNode)
    {
        // explore by type
        switch (pNode->GetPrincipalInterfaceType())
        {
        case GenApi::intfIBoolean:
            exploreBoolean(pNode);
            break;
        case GenApi::intfIString:
            exploreString(pNode);
            break;
        case GenApi::intfIEnumeration:
            exploreEnumeration(pNode);
            break;
        case GenApi::intfIInteger:
            exploreInteger(pNode);
            break;
        case GenApi::intfIFloat:
            exploreFloat(pNode);
            break;
        default:
            LOG_DEBUG << "ArenaSDKManager::exploreNode;" << TAB3 << nodeName << " type not found\n";
        }
    }
    else
    {
        LOG_DEBUG << "ArenaSDKManager::exploreNode;" << TAB3 << nodeName << " not found\n";
    }


    // print node information
    LOG_DEBUG << "ArenaSDKManager::exploreNode;" << Indent(depth) << displayName << " (" << nodeName << ")" << std::setw(90 - displayName.size() - nodeName.size() - (depth * 2)) << " ";

    if (EXPLORE_ACCESS)
        LOG_DEBUG << "ArenaSDKManager::exploreNode;" << accessModeStr << std::setw(5) << " ";

    if (EXPLORE_VISIBILITY)
        LOG_DEBUG << "ArenaSDKManager::exploreNode;" << visibilityStr << std::setw(14 - visibilityStr.size()) << " ";

    if (EXPLORE_TYPE)
        LOG_DEBUG << "ArenaSDKManager::exploreNode;" << interfaceTypeStr << std::setw(20 - interfaceTypeStr.size()) << " ";

    if (EXPLORE_VALUE)
        LOG_DEBUG << "ArenaSDKManager::exploreNode;" << (value.size() < 90 ? value : "...");

    LOG_DEBUG << "ArenaSDKManager::exploreNode;" << "\n";

    // Explore category node children
    GenApi::CCategoryPtr pCategory = pNode;
    if (pCategory)
    {
        GenApi::FeatureList_t children;
        pCategory->GetFeatures(children);
        for (GenApi::CValuePtr pValue : children)
            exploreNode(pValue->GetNode(), depth + 1);
    }
}

void ArenaSDKManager::exploreBoolean(GenApi::CBooleanPtr pBoolean)
{
    try {
        if (!pBoolean.IsValid()) {
            LOG_DEBUG << "ArenaSDKManager::exploreBoolean;" << TAB3 << "NODE IS NOT VALID";
            return;
        }

        // Retrieve value
        LOG_DEBUG << "ArenaSDKManager::exploreBoolean;" << TAB3 << "Value: ";

        bool value = pBoolean->GetValue();

        LOG_DEBUG << "ArenaSDKManager::exploreBoolean;" << value << "\n";

        // Set value
        //    {
        // bool value = true; pBoolean->SetValue(value);
        //    }


    }
    catch (GenICam::GenericException& ge)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreBoolean;" << ge.what();
    }
    catch (std::exception& ex)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreBoolean;" << ex.what();
    }
    catch (...)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreBoolean;" << "Unexpected exception thrown";
    }
}

void ArenaSDKManager::exploreString(GenApi::CStringPtr pString)
{
    try{
        if (!pString.IsValid()) {
            LOG_DEBUG << "ArenaSDKManager::exploreString;" << TAB3 << "NODE IS NOT VALID";
            return;
        }

    // Retrieve value
    LOG_DEBUG << "ArenaSDKManager::exploreString;" << TAB3 << "Value: ";

    GenICam::gcstring value = pString->GetValue();

    LOG_DEBUG << "ArenaSDKManager::exploreString;" << value << "\n";

    // Retrieve maximum length
    LOG_DEBUG << "ArenaSDKManager::exploreString;" << TAB3 << "Maximum length: ";

    int64_t maxLen = pString->GetMaxLength();

    LOG_DEBUG << "ArenaSDKManager::exploreString;" << maxLen << "\n";

    // Set value
    //    {
    //        GenICam::gcstring value = "string value";
    //    pString->SetValue(value);
    //    }


    }
    catch (GenICam::GenericException& ge)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreString;" << ge.what();
    }
    catch (std::exception& ex)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreString;" << ex.what();
    }
    catch (...)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreString;" << "Unexpected exception thrown";
    }
}

void ArenaSDKManager::exploreEnumeration(GenApi::CEnumerationPtr pEnumeration)
{
    try {
        if (!pEnumeration.IsValid()) {
            LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << TAB3 << "NODE IS NOT VALID";
            return;
        }

    // Retrieve current entry
    LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << TAB3 << "Current entry: ";

    GenApi::CEnumEntryPtr pCurrentEntry = pEnumeration->GetCurrentEntry();
    GenICam::gcstring currentEntrySymbolic = pCurrentEntry->GetSymbolic();

    LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << currentEntrySymbolic << "\n";

    // Retrieve integer value
    LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << TAB3 << "Integer value: ";

    int64_t intValue = pEnumeration->GetIntValue();

    LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << intValue << "\n";

    // Retrieve entries
    LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << TAB3 << "Entries: ";

    GenApi::NodeList_t entries;
    pEnumeration->GetEntries(entries);

    for (size_t i = 0; i < entries.size(); i++)
    {
        if (i > 0)
            std::cout << ", ";

        GenApi::CEnumEntryPtr pEntry = entries[i];
        std::cout << pEntry->GetSymbolic();
    }
    std::cout << "\n";

    // Retrieve symbolics
    //    {
    //        GenApi::StringList_t symbolics;
    //    pEnumeration->GetSymbolics(symbolics);
    //    }

    // Retrieve entry by name
    //    {
    //        GenICam::gcstring symbolic = "symbolic value";
    //    GenApi::CEnumEntryPtr pEntry = pEnumeration->GetEntryByName(symbolic);
    //    }

    // Set value
    //    {
    //        GenICam::gcstring symbolic = "symbolic value";
    //    GenApi::CEnumEntryPtr pEntry = pEnumeration->GetEntryByName(symbolic);
    //    int64_t intValue = pEntry->GetValue();
    //    pEnumeration->SetIntValue(intValue);
    //    }


    }
    catch (GenICam::GenericException& ge)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << ge.what();
    }
    catch (std::exception& ex)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << ex.what();
    }
    catch (...)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreEnumeration;" << "Unexpected exception thrown";
    }
}

void ArenaSDKManager::exploreInteger(GenApi::CIntegerPtr pInteger)
{
    try {
        if (!pInteger.IsValid()) {
            LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << TAB3 << "NODE IS NOT VALID";
            return;
        }

        // Retrieve value
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << TAB3 << "Value: ";

        int64_t value = pInteger->GetValue();

        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << value << "\n";

        // Retrieve range
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << TAB3 << "Minimum, maximum: ";

        int64_t min = pInteger->GetMin();
        int64_t max = pInteger->GetMax();

        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << min << ", " << max << "\n";

        // Retrieve increment
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << TAB3 << "Increment (mode): ";

        int64_t inc = pInteger->GetInc();
        GenApi::EIncMode incMode = pInteger->GetIncMode();
        GenICam::gcstring incModeStr = Arena::EIncModeClass::ToString(incMode);

        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << inc << " (" << incModeStr << ")\n";

        // Retrieve representation
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << TAB3 << "Representation: ";

        GenApi::ERepresentation rep = pInteger->GetRepresentation();
        GenICam::gcstring repStr = GenApi::ERepresentationClass::ToString(rep);

        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << repStr << "\n";

        // Retrieve unit
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << TAB3 << "Unit: ";

        GenICam::gcstring unit = pInteger->GetUnit();

        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << unit << "\n";

        // Impose maximum and minimum
        //    {
        // int64_t newMax = 10; int64_t newMin = 0; pInteger->ImposeMax(newMax);
        // pInteger->ImposeMin(newMin);
        //    }

        // Set value
        //    {
        // int64_t value = 0; pInteger->SetValue(value);
        //    }

    }
    catch (GenICam::GenericException& ge)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << ge.what();
    }
    catch (std::exception& ex)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << ex.what();
    }
    catch (...)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << "Unexpected exception thrown";
    }
}

void ArenaSDKManager::exploreFloat(GenApi::CFloatPtr pFloat)
{
    try {
        if (!pFloat.IsValid()) {
            LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << TAB3 << "NODE IS NOT VALID";
            return;
        }

        // Retrieve value
        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << TAB3 << "Value: ";

        double value = pFloat->GetValue();

        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << value << "\n";

        // Retrieve range
        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << TAB3 << "Minimum, maximum: ";

        double min = pFloat->GetMin();
        double max = pFloat->GetMax();

        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << min << ", " << max << "\n";

        // Retrieve increment
        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << TAB3 << "Increment (mode): ";

        bool hasIncrement = pFloat->HasInc();

        if (hasIncrement)
        {
            double inc = pFloat->GetInc();
            GenApi::EIncMode incMode = pFloat->GetIncMode();
            GenICam::gcstring incModeStr = Arena::EIncModeClass::ToString(incMode);

            LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << inc << " (" << incModeStr << ")\n";
        }
        else
        {
            LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << "no increment\n";
        }

        // Retrieve representation
        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << TAB3 << "Representation: ";

        GenApi::ERepresentation rep = pFloat->GetRepresentation();
        GenICam::gcstring repStr = GenApi::ERepresentationClass::ToString(rep);

        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << repStr << "\n";

        // Retrieve unit
        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << TAB3 << "Unit: ";

        GenICam::gcstring unit = pFloat->GetUnit();

        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << unit << "\n";

        // Retrieve display notation
        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << TAB3 << "Display notation: ";

        GenApi::EDisplayNotation dispNotation = pFloat->GetDisplayNotation();
        GenICam::gcstring dispNotationStr = GenApi::EDisplayNotationClass::ToString(dispNotation);

        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << dispNotationStr << "\n";

        // Retrieve display precision
        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << TAB3 << "Display precision: ";

        int64_t dispPrecision = pFloat->GetDisplayPrecision();

        LOG_DEBUG << "ArenaSDKManager::exploreFloat;" << dispPrecision << "\n";

        // Impose maximum and minimum
        //    {
        // double newMax = 10; double newMin = 0; pFloat->ImposeMax(newMax);
        // pFloat->ImposeMin(newMin);
        //    }

        // Set value
        //    {
        // double value = 0; pFloat->SetValue(value);
        //   

    }
    catch (GenICam::GenericException& ge)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << ge.what();
    }
    catch (std::exception& ex)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << ex.what();
    }
    catch (...)
    {
        LOG_DEBUG << "ArenaSDKManager::exploreInteger;" << "Unexpected exception thrown";
    }
}
#endif