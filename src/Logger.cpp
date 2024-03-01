#include "Logger.h"

#include "Logger_Boost.h"
#include "Logger_Log4Cpp.h"
#include "Logger_Console.h"

using namespace std;
using namespace freeture;

shared_ptr<ILogger> Logger::m_Instance;
shared_ptr<Logger> Logger::m_LoggerInstance;
mutex Logger::m_LoggerInstanceMutex;

// src::sever
Logger::Logger(LogThread thread, thread_id_type thread_id, std::string log_path_folder, int log_archive_days, int log_archive_limit_mb, LogSeverityLevel log_severity) :
    m_LogPathFolder(log_path_folder),
    m_LogArchiveDays(log_archive_days),
    m_LogArchiveLimitMB(log_archive_limit_mb),
    m_LogSeverityLevel(log_severity),
    m_LogMap()
{
    if (m_Instance == nullptr) {
        if (USE_BOOST_LOG)
        {
            m_Instance = make_shared<Logger_Boost>(m_LoggerInstance);
        }
        else
        {
            m_Instance = make_shared<Logger_Log4Cpp>(m_LoggerInstance);
        }
    }

    if (m_Instance->logger == nullptr)
    {
        m_Instance->logger = m_LoggerInstance;
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
            m_Instance = make_shared<Logger_Boost>(m_LoggerInstance);
        }
        else
        {
            m_Instance = make_shared<Logger_Log4Cpp>(m_LoggerInstance);
        }
    }

    m_Instance->level(level);
    return *m_Instance.get();
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
}

shared_ptr<Logger> Logger::Get()
{
    if (!m_LoggerInstance)
        throw runtime_error("Logger is not initialized");

    return m_LoggerInstance;
}

shared_ptr<Logger> Logger::Get(LogThread log_thread, thread_id_type thread_id, string log_path_folder, int log_archive_days, int log_archive_limit_mb, LogSeverityLevel log_severity)
{
    std::lock_guard<std::mutex> lock(m_LoggerInstanceMutex);

    if (!m_LoggerInstance)
    {
        m_LoggerInstance = std::shared_ptr<Logger>(new Logger(log_thread, thread_id, log_path_folder, log_archive_days, log_archive_limit_mb, log_severity));
    }
    
    if (m_Instance->logger == nullptr)
    {
        m_Instance->logger = m_LoggerInstance;
        m_Instance->init();
    }

    return m_LoggerInstance;
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

LogThread Logger::getLogThread(thread_id_type thread_id)
{
    std::ostringstream oss;
    oss << thread_id;
    std::string threadIdStr = oss.str();
    for (const auto& pair : m_LogMap) {
        if (pair.second == threadIdStr) {
            return pair.first;
        }
    }

    return LogThread::UNDEFINED;
}
