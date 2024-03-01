#include "Logger_Log4Cpp.h"

#include "Logger.h"


#include <fstream> // For std::ifstream and std::ofstream
#include <sstream> // For std::istringstream (if needed)
#include <string>  // For std::string
#include <iostream> // For std::cout, used in the logging examples

#include <log4cpp/Category.hh>
#include <log4cpp/Appender.hh>
#include <log4cpp/FileAppender.hh>
#include <log4cpp/PropertyConfigurator.hh>
#include <log4cpp/RollingFileAppender.hh>

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp> // For efficient file I/O

using namespace std;
using namespace freeture;
using namespace log4cpp_freeture;

void Logger_Log4Cpp::updateAppenderConfiguration(string appender_name) {
    // Assume "A1" is the name of the RollingFileAppender
    Appender* appender = Category::getRoot().getAppender(appender_name);
    RollingFileAppender* rollingFileAppender = dynamic_cast<RollingFileAppender*>(appender);

    if (rollingFileAppender != nullptr)
    {
        rollingFileAppender->setMaxBackupIndex(logger->getLogArchiveDays()); // Change the number of backup files
        rollingFileAppender->setMaximumFileSize(1024 * 1024 * logger->getLogArchiveLimitMB()); // Change max file size to 10MB
    }
}

void Logger_Log4Cpp::setRootLogLevel(Priority::Value level) {
    Category& root = Category::getRoot();
    root.setPriority(level);
}


Logger_Log4Cpp::Logger_Log4Cpp(shared_ptr<Logger> logger):
    ILogger(logger)
{
    
}

void Logger_Log4Cpp::apply() {
    updateAppenderConfiguration("FREETURE");
    updateAppenderConfiguration("ACQ_THREAD");
    updateAppenderConfiguration("DET_THREAD");
    updateAppenderConfiguration("STACK_THREAD");

    switch (logger->getLogSeverityLevel()) {
    case LogSeverityLevel::critical:
        setRootLogLevel(Priority::CRIT);
        break;
    case LogSeverityLevel::fail:
        setRootLogLevel(Priority::ERROR);
        break;
    case LogSeverityLevel::normal:
        setRootLogLevel(Priority::DEBUG);
        break;
    case LogSeverityLevel::notification:
        setRootLogLevel(Priority::INFO);
        break;
    case LogSeverityLevel::warning:
        setRootLogLevel(Priority::WARN);
        break;
    };
}

void Logger_Log4Cpp::preprocessAndLoadConfiguration(const std::string& configFile) {
    string base_log_dir = logger->getLogPathFolder();
    boost::filesystem::path configPath(configFile);

    if (!boost::filesystem::exists(configPath)) {
        throw runtime_error( "Configuration file does not exist: " + configFile );
        return;
    }
    boost::iostreams::mapped_file_source file(configFile);
    std::string configContent(file.data(), file.size());

    // Replace placeholder with actual directory path
    size_t pos = 0;
    while ((pos = configContent.find("%BASE_LOG_DIR%", pos)) != std::string::npos) {
        configContent.replace(pos, std::strlen("%BASE_LOG_DIR%"), base_log_dir);
        pos += base_log_dir.length();
    }

    // Temporary file to load the modified configuration
    boost::filesystem::path tempConfigPath = configPath.parent_path() / "freeture_log4cpp.properties";
    boost::iostreams::mapped_file_params params;
    params.path = tempConfigPath.string();
    params.new_file_size = configContent.size(); // Set the size of the new file
    params.flags = boost::iostreams::mapped_file::mapmode::readwrite;

    boost::iostreams::mapped_file_sink out(params);
    std::copy(configContent.begin(), configContent.end(), out.data());
    out.close();
}

void Logger_Log4Cpp::init()
{
    if (!logger)
        return;
    

    try {
        preprocessAndLoadConfiguration(logger->getLogPathFolder() + "log4cpp.properties");
        // Initialize Log4cpp here, potentially loading configuration from a file
        PropertyConfigurator::configure(logger->getLogPathFolder() + "freeture_log4cpp.properties");

        apply();
        fetchCategories();

        m_Init = true;
    }
    catch (exception& e) {
        throw runtime_error ( e.what() );
    }
}

void Logger_Log4Cpp::logDebug(const std::string& message) {
    Category& category = getCategory();
    
    category.debug(message);
}

void Logger_Log4Cpp::logNotification(const std::string& message) {
    Category& category = getCategory();
    
    category.info(message);
}

void Logger_Log4Cpp::logCritical(const std::string& message)
{
    Category& category = getCategory();

    category.error(message);
}

void Logger_Log4Cpp::logFail(const std::string& message)
{
    Category& category = getCategory();

    category.error(message);
}

void Logger_Log4Cpp::logNormal(const std::string& message)
{
    Category& category = getCategory();

    category.debug(message);
}


void Logger_Log4Cpp::logWarning(const std::string& message)
{
    Category& category = getCategory();

    category.warn(message);
}

Category& Logger_Log4Cpp::getCategory()
{
    auto threadId = std::this_thread::get_id();

     if (logger == nullptr)
         return *m_FreetureCategory;

    switch (logger->getLogThread(threadId))
    {
    case LogThread::DETECTION_THRED:
        return *m_DetThreadCategory;

    case LogThread::ACQUISITION_THREAD:
        return *m_AcqThreadCategory;

    case LogThread::STACK_THREAD:
        return *m_StackThreadCategory;

    case LogThread::UNDEFINED:
    case LogThread::FREETURE_THREAD:
        return *m_FreetureCategory;
    }
}


void Logger_Log4Cpp::fetchCategories()
{
    std::lock_guard<std::mutex> guard(m_RecordMutex);
    
    m_AcqThreadCategory     = &Category::getInstance("ACQ_THREAD");
    m_StackThreadCategory   = &Category::getInstance("STACK_THREAD");
    m_FreetureCategory      = &Category::getInstance("FREETURE");
    m_DetThreadCategory     = &Category::getInstance("DET_THREAD");
}


void Logger_Log4Cpp::attach(LogThread, thread_id_type)
{

}

void Logger_Log4Cpp::reset()
{

}

void Logger_Log4Cpp::flush(std::string message)
{
    if (!m_Init)
        return;

    if (message.empty())
        return;

    switch (m_LogLevel) {
    case LogSeverityLevel::normal: {
        logNormal(m_Stream.str());
        break;
    }
    case LogSeverityLevel::notification: {
        logNotification(m_Stream.str());
        break;
    }
    case LogSeverityLevel::fail: {
        logFail(m_Stream.str());
        break;
    }
    case LogSeverityLevel::critical: {
        logCritical(m_Stream.str());
        break;
    }
    case LogSeverityLevel::warning: {
        logWarning(m_Stream.str());
        break;
    }
    };
}

