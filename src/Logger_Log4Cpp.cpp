#include "Logger_Log4Cpp.h"

#include "Logger.h"

#include <log4cpp/Appender.hh>
#include <log4cpp/FileAppender.hh>
#include <log4cpp/PropertyConfigurator.hh>

using namespace std;
using namespace freeture;
using namespace log4cpp;

Logger_Log4Cpp::Logger_Log4Cpp(Logger* logger):
    ILogger(logger)
{
    
}

void Logger_Log4Cpp::init() {
    // Initialize Log4cpp here, potentially loading configuration from a file
    PropertyConfigurator::configure("log4cpp.properties");
}


void Logger_Log4Cpp::logDebug(const std::string& message) {
    getCategory().debug(message);
}

void Logger_Log4Cpp::logInfo(const std::string& message) {
    getCategory().info(message);
}

void Logger_Log4Cpp::logError(const std::string& message)
{
    getCategory().error(message);
}

void Logger_Log4Cpp::logFatal(const std::string& message)
{
    getCategory().error(message);
}


Category& Logger_Log4Cpp::getCategory() {
    std::lock_guard<std::mutex> guard(mutex);
    auto threadId = std::this_thread::get_id();
    if (categories.find(threadId) == categories.end()) {
        std::ostringstream oss;
        oss << "Thread-" << threadId;
        auto& category = Category::getInstance(oss.str());
        categories[threadId] = &category;
        // Configure the category for this thread, e.g., setting up appender to write to a file named after threadId
    }
    return *categories[threadId];
}

void Logger_Log4Cpp::setLogThread(LogThread, thread_id_type, bool reset )
{

}

void Logger_Log4Cpp::attach(LogThread, thread_id_type)
{

}

void Logger_Log4Cpp::reset()
{

}

