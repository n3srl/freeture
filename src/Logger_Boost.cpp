#include "Logger_Boost.h"

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

logger_type freeture::Logger_Boost::m_LoggerInstance;

//src::severity_channel_logger_mt< trivial::severity_level > freeture::Logger::m_LoggerInstance;
// src::severity_channel_logger_mt< trivial::severity_level > freeture::Logger::m_DetThreadLogger;
// src::severity_channel_logger_mt< trivial::severity_level > freeture::Logger::m_StackThreadLogger;
// src::severity_channel_logger_mt< trivial::severity_level > freeture::Logger::m_AcqThreadLogger;

freeture::Logger_Boost::Logger_Boost(shared_ptr<Logger>  logger) :
    ILogger(logger)
{
}

void freeture::Logger_Boost::addDefaultSink(string logger_name, string file_name, LogThread thread)
{
    boost::shared_ptr< logging::core > core = logging::core::get();
    boost::shared_ptr< backend_type > m_SinkBackend;
    boost::shared_ptr< sink_type > m_Sink;

    m_SinkBackend = boost::make_shared<backend_type>(
        keywords::target = logger->getLogPathFolder(),
        keywords::file_name = logger->getLogPathFolder() + string(file_name),
        keywords::rotation_size = logger->getLogArchiveLimitMB() * 1024 * 1024,
        keywords::time_based_rotation = sinks::file::rotation_at_time_interval(posix_time::hours(logger->getLogArchiveDays() * 24)),
        keywords::auto_flush = true,
        keywords::format = m_LogString,
        keywords::max_size = logger->getLogArchiveLimitMB() * 1024 * 1024 //bytes 
        );

    m_Sink = boost::make_shared<sink_type>(m_SinkBackend);

    if (logger->hasThreadId(thread)) {
        string thread_id = logger->getThreadId(thread);
       // m_Sink->set_filter(trivial::severity >= m_LogSeverityFilter && expr::attr<string>("ThreadTag")  == thread_id);
        m_Sink->set_filter(trivial::severity >= m_LogSeverityFilter && expr::attr<std::string>("Channel") == logger->getThreadId());
    }
    else
        m_Sink->set_filter(trivial::severity >= m_LogSeverityFilter);

    m_Sink->set_formatter(logging::parse_formatter(m_LogString));
    core->add_sink(m_Sink);
}

void freeture::Logger_Boost::reset() {
    logging::core::get()->remove_all_sinks();
}

void freeture::Logger_Boost::flush(std::string message)
{

}

void freeture::Logger_Boost::attach(LogThread log_thread, thread_id_type thread_id)
{
    std::ostringstream oss;
    oss << thread_id;
    std::string threadIdStr = oss.str();

    logging::core::get()->add_thread_attribute("ThreadTag", attrs::constant<string>(threadIdStr));

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

void freeture::Logger_Boost::setSeverityLevel()
{
    switch (logger->getLogSeverityLevel())
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


void freeture::Logger_Boost::init()
{
    setSeverityLevel();

    logging::add_common_attributes();
    logging::core::get()->add_global_attribute("ThreadID", attrs::current_thread_id());

    logging::add_console_log(std::cout, boost::log::keywords::format = m_LogString);

}

logger_type& freeture::Logger_Boost::get()
{
    return m_LoggerInstance;
}