#include "Logger.h"

#include <boost/log/expressions/keyword_fwd.hpp>
#include <boost/log/expressions/is_keyword_descriptor.hpp>
#include <boost/log/expressions/attr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> //include all types plus i/o

namespace expr = boost::log::expressions;
namespace attrs = boost::log::attributes;
namespace posix_time = boost::posix_time;

using namespace std;

// shared_ptr<freeture::Logger> freeture::Logger::m_Instance = nullptr;
// mutex freeture::Logger::m_Mutex;

logger_type freeture::Logger::m_LoggerInstance;

//src::severity_channel_logger_mt< trivial::severity_level > freeture::Logger::m_LoggerInstance;
// src::severity_channel_logger_mt< trivial::severity_level > freeture::Logger::m_DetThreadLogger;
// src::severity_channel_logger_mt< trivial::severity_level > freeture::Logger::m_StackThreadLogger;
// src::severity_channel_logger_mt< trivial::severity_level > freeture::Logger::m_AcqThreadLogger;

freeture::Logger* freeture::Logger::m_Instance;
// src::sever
freeture::Logger::Logger(freeture::LogThread thread,thread_id_type thread_id, std::string log_path_folder, int log_archive_days, int log_archive_limit_mb, LogSeverityLevel log_severity) :
    m_LogThread(thread),
    m_LogPathFolder(log_path_folder),
    m_LogArchiveDays(log_archive_days),
    m_LogArchiveLimitMB(log_archive_limit_mb),
    m_LogSeverityLevel(log_severity)
{
    m_Instance = this;

    init();

    setLogThread(thread, thread_id);
}

void freeture::Logger::addDefaultSink(string logger_name, string file_name, LogThread thread)
{
    boost::shared_ptr< logging::core > core = logging::core::get();
    boost::shared_ptr< backend_type > m_SinkBackend;
    boost::shared_ptr< sink_type > m_Sink;

    m_SinkBackend = boost::make_shared<backend_type>(
        keywords::target = m_LogPathFolder,
        keywords::file_name = m_LogPathFolder + string(file_name),
        keywords::rotation_size = m_LogArchiveLimitMB * 1024 * 1024,
        keywords::time_based_rotation = sinks::file::rotation_at_time_interval(posix_time::hours( m_LogArchiveDays * 24)),
        keywords::auto_flush = true,
        keywords::format = m_LogString,
        keywords::max_size = m_LogArchiveLimitMB * 1024 * 1024 //bytes 
        );

    m_Sink = boost::make_shared<sink_type>(m_SinkBackend);

    if (hasThreadId(thread)) {
        string thread_id = getThreadId(thread);
       // m_Sink->set_filter(trivial::severity >= m_LogSeverityFilter && expr::attr<string>("ThreadTag")  == thread_id);
        m_Sink->set_filter(trivial::severity >= m_LogSeverityFilter && expr::attr<std::string>("Channel") == getThreadId());
    }
    else
        m_Sink->set_filter(trivial::severity >= m_LogSeverityFilter);

    m_Sink->set_formatter(logging::parse_formatter(m_LogString));
    core->add_sink(m_Sink);
}

bool freeture::Logger::hasThreadId(LogThread thread)
{
    return (m_LogMap.find(thread) != m_LogMap.end());
}

string freeture::Logger::getThreadId(LogThread thread)
{
    if (m_LogMap.find(thread) != m_LogMap.end()) 
        return m_LogMap[thread];

    return string();
}

void freeture::Logger::setLogThread(LogThread log_thread, thread_id_type thread_id, bool reset)
{
    std::ostringstream oss;
    oss << thread_id;
    std::string threadIdStr = oss.str();

    logging::core::get()->add_thread_attribute("ThreadTag", attrs::constant<string>(threadIdStr));

    if ( m_LogMap.find(log_thread) == m_LogMap.end() )
    {
        m_LogMap[log_thread] = threadIdStr;

        if (reset)
        {
            logging::core::get()->remove_all_sinks();
        }
    
        switch (log_thread) {
        case LogThread::FREETURE_THREAD:
            addDefaultSink(FREETURE_LOGGER_NAME, FREETURE_FILE_NAME, LogThread::FREETURE_THREAD);
            break;
        case LogThread::ACQUISITION_THREAD:
            addDefaultSink(ACQ_THREAD_LOGGER_NAME, ACQ_THREAD_FILE_NAME, LogThread::ACQUISITION_THREAD);
            break;
        case LogThread::STACK_THREAD:
            addDefaultSink(STACK_THREAD_LOGGER_NAME, STACK_THREAD_FILE_NAME, LogThread::STACK_THREAD);
            break;
        case LogThread::DETECTION_THRED:
            addDefaultSink(DET_THREAD_LOGGER_NAME, DET_THREAD_FILE_NAME, LogThread::DETECTION_THRED);
            break;
        };
    }
}

string freeture::Logger::getThreadId()
{
    std::ostringstream oss;
    oss << std::this_thread::get_id();
    string threadIdStr = oss.str();

    return threadIdStr;
}

void freeture::Logger::setSeverityLevel()
{
    switch (m_LogSeverityLevel)
    {
    case LogSeverityLevel::critical:
        m_LogSeverityFilter = trivial::fatal;
        break;
    case LogSeverityLevel::fail:
        m_LogSeverityFilter = trivial::error;
        break;
    case LogSeverityLevel::warning:
        m_LogSeverityFilter = trivial::warning;
        break;
    case LogSeverityLevel::notification:
        m_LogSeverityFilter = trivial::info;
        break;
    case LogSeverityLevel::normal:
        m_LogSeverityFilter = trivial::debug;
        break;
    };
}


void freeture::Logger::init()
{
    setSeverityLevel();

    logging::add_common_attributes();
    logging::core::get()->add_global_attribute("ThreadID", attrs::current_thread_id());

    logging::add_console_log(std::cout, boost::log::keywords::format = m_LogString);

}

// 
// shared_ptr<freeture::Logger> freeture::Logger::Get(string log_path_folder, int log_archive_days, int log_archive_limit_mb, LogSeverityLevel log_severity)
// {
//     lock_guard<mutex> lock(m_Mutex);
// 
//     if (!m_Instance)
//         m_Instance = make_shared<Logger>(log_path_folder, log_archive_days, log_archive_limit_mb, log_severity);
// 
//     return m_Instance;
// }

logger_type& freeture::Logger::get()
{
    return m_LoggerInstance;
}

freeture::Logger* freeture::Logger::GetLogger()
{
    return m_Instance;
}

