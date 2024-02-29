#include "Logger.h"
#include "Logger_Boost.h"
#include "Logger_Log4Cpp.h"
#include "Logger_Console.h"

using namespace std;
using namespace freeture;

ILogger* Logger::m_Instance;
Logger* Logger::m_LoggerInstance;

// src::sever
Logger::Logger(LogThread thread, thread_id_type thread_id, std::string log_path_folder, int log_archive_days, int log_archive_limit_mb, LogSeverityLevel log_severity) :
    m_LogPathFolder(log_path_folder),
    m_LogArchiveDays(log_archive_days),
    m_LogArchiveLimitMB(log_archive_limit_mb),
    m_LogSeverityLevel(log_severity),
    m_LogMap()
{
    m_LoggerInstance = this;
   
    if (m_Instance == nullptr) {


        if (USE_BOOST_LOG)
        {
            m_Instance = new Logger_Boost(m_LoggerInstance);
        }
        else
        {
            m_Instance = new Logger_Log4Cpp(m_LoggerInstance);
        }
    }

    m_Instance->init();
}

bool Logger::hasThreadId(LogThread thread)
{
    return (m_LogMap.find(thread) != m_LogMap.end());
}

string Logger::getThreadId(LogThread thread)
{
    if (m_LogMap.find(thread) != m_LogMap.end()) 
        return m_LogMap[thread];

    return string();
}

string Logger::getThreadId()
{
    std::ostringstream oss;
    oss << std::this_thread::get_id();
    string threadIdStr = oss.str();

    return threadIdStr;
}


ILogger& Logger::GetLogger(LogSeverityLevel level)
{
    if (m_Instance == nullptr)
    {
        if (USE_BOOST_LOG)
        {
            m_Instance = new Logger_Boost(m_LoggerInstance);
        }
        else
        {
            m_Instance = new Logger_Log4Cpp(m_LoggerInstance);
        }
    }

    m_Instance->level(level);
    return *m_Instance;
}

void Logger::setLogThread(LogThread log_thread, thread_id_type thread_id, bool reset_logger)
{
    std::ostringstream oss;
    oss << thread_id;
    std::string threadIdStr = oss.str();

    if (m_LogMap.find(log_thread) == m_LogMap.end())
    {
        m_LogMap[log_thread] = threadIdStr;

        if (reset_logger)
        {
            m_Instance->reset();
        }

        m_Instance->attach( log_thread, thread_id );
    }
}

Logger::~Logger()
{
    delete m_Instance;
}

Logger& Logger::Get()
{
    return *m_LoggerInstance;
}

freeture::LogSeverityLevel Logger::getLogSeverityLevel()
{
    return m_LogSeverityLevel;
}

int Logger::getLogArchiveDays()
{
    return m_LogArchiveDays;
}

int Logger::getLogArchiveLimitMB()
{
    return m_LogArchiveLimitMB;
}

std::string Logger::getLogPathFolder()
{
    return m_LogPathFolder;
}
