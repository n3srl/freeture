#pragma once
/**
* \file    Logger.h
* \author   Andrea Novati - N3 s.r.l.
* \version 1.0
* \date    11/02/2024
*/
#include "Commons.h"
#include "ILogger.h"

#include "ELogSeverityLevel.h"
#include "ELogThread.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>


#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/attributes/current_thread_id.hpp>
#include <boost/log/expressions/keyword_fwd.hpp>
#include <boost/log/expressions/keyword.hpp>
#include <boost/log/keywords/rotation_size.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sinks/text_file_backend.hpp>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;
namespace attrs = boost::log::attributes;
namespace trivial = boost::log::trivial;
namespace fs = boost::filesystem;

using backend_type = sinks::text_file_backend;
using sink_type = sinks::synchronous_sink<backend_type>;
using logger_type = src::severity_channel_logger_mt<trivial::severity_level>;

#define BOOST_LOG_DYN_LINK 1

BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", freeture::LogSeverityLevel)
BOOST_LOG_ATTRIBUTE_KEYWORD(channel,  "Channel", std::string)
BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", freeture::LogThread)

// 
// BOOST_LOG_GLOBAL_LOGGER(freeture_logger, src::severity_logger_mt< trivial::severity_level >)
// BOOST_LOG_GLOBAL_LOGGER(det_thread_logger, src::severity_logger_mt< trivial::severity_level >)
// BOOST_LOG_GLOBAL_LOGGER(stack_thread_logger, src::severity_logger_mt< trivial::severity_level >)
// BOOST_LOG_GLOBAL_LOGGER(acq_thread_logger, src::severity_logger_mt< trivial::severity_level >)

// BOOST_LOG_GLOBAL_LOGGER_INIT(freeture_logger, boost::log::sources::severity_logger_mt<trivial::severity_level>)
// {
//     boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger;
//     logger.add_attribute("LoggerName", boost::log::attributes::constant<std::string>(FREETURE_LOGGER_NAME));
//     return logger;
// }

// 
// BOOST_LOG_GLOBAL_LOGGER_INIT(det_thread_logger, boost::log::sources::severity_logger_mt<trivial::severity_level>)
// {
//     boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger;
//     logger.add_attribute("LoggerName", boost::log::attributes::constant<std::string>(DET_THREAD_LOGGER_NAME));
//     return logger;
// }
// 
// BOOST_LOG_GLOBAL_LOGGER_INIT(stack_thread_logger, boost::log::sources::severity_logger_mt<trivial::severity_level>)
// {
//     boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger;
//     logger.add_attribute("LoggerName", boost::log::attributes::constant<std::string>(STACK_THREAD_LOGGER_NAME));
//     return logger;
// }
// 
// BOOST_LOG_GLOBAL_LOGGER_INIT(acq_thread_logger, boost::log::sources::severity_logger_mt<trivial::severity_level>)
// {
//     boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger;
//     logger.add_attribute("LoggerName", boost::log::attributes::constant<std::string>(ACQ_THREAD_LOGGER_NAME));
//     return logger;
// }

#define DET_THREAD_LOGGER_NAME "det_thread_logger"
#define STACK_THREAD_LOGGER_NAME "stack_thread_logger"
#define ACQ_THREAD_LOGGER_NAME "acq_thread_logger"
#define FREETURE_LOGGER_NAME "freeture_thread_logger"

#define DET_THREAD_FILE_NAME "DET_THREAD.%N.log"
#define STACK_THREAD_FILE_NAME "STACK_THREAD.%N.log"
#define ACQ_THREAD_FILE_NAME "ACQ_THREAD.%N.log"
#define FREETURE_FILE_NAME "FREETURE.%N.log"


static logger_type g_logger;
// 

namespace freeture
{
  
    class Logger_Boost: public ILogger
    {

    public:
        Logger_Boost(Logger*);

        void init() override;
        void attach(LogThread, thread_id_type) override;
        void reset() override;

    private:
        std::string m_LogString = "%TimeStamp%;ThreadID: %ThreadID%;%Severity%;%Tag%; %Message%";

        static logger_type m_LoggerInstance;
        trivial::severity_level m_LogSeverityFilter;
    private:
        /// <summary>
        ///   trace - The finest level of granularity, used for detailed debugging information that may be useful in diagnosing problems.
        //    debug - Informational messages targeted at developers or system administrators, useful for debugging.
        //    info - General information about application progress or state, typically non - critical but useful for general operation insights.
        //    warning - Potentially harmful situations indicating something unexpected or a potential problem that should be looked into.
        //    error - Error events of considerable importance that will presumably lead to an abrupt application cessation or a significant malfunction.
        //    fatal - Very severe error events that will presumably lead the application to abort.
        /// </summary>
        void setSeverityLevel();

        void addDefaultSink(std::string, std::string, LogThread);
        static logger_type& get();

    };
}
 