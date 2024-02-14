#pragma once
/**
* \file    Logger.h
* \author   Andrea Novati - N3 s.r.l.
* \version 1.0
* \date    11/02/2024
*/
#include "Commons.h"

#include <fstream>
#include <string>
#include <memory>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#include <boost/log/attributes/scoped_attribute.hpp>

#include <boost/log/expressions/keyword_fwd.hpp>
#include <boost/log/expressions/keyword.hpp>
#include "boost/log/keywords/rotation_size.hpp"

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;
namespace attrs = boost::log::attributes;
namespace trivial = boost::log::trivial;

#define BOOST_LOG_DYN_LINK 1

BOOST_LOG_INLINE_GLOBAL_LOGGER_DEFAULT(freeture_logger, src::severity_logger_mt< trivial::severity_level >)

typedef sinks::synchronous_sink< sinks::text_ostream_backend  > text_sink;

namespace freeture
{
    class Logger
    {
        public:

            static Logger& Get()
            {
                static Logger instance;

                return instance;
            }

        private: 

           Logger()
           {
               init();
               add_common_attributes();
           }
    public:
           void add_common_attributes()
           {
               boost::shared_ptr< logging::core > core = logging::core::get();
               core->add_global_attribute("TimeStamp", attrs::local_clock());
           }

            void init()
            {
                std::string log_string = "[%TimeStamp%];<%Severity%>;(%ClassName%); %Message%";

                logging::add_file_log
                (
                    keywords::file_name = "freeture_%N.log",
                    keywords::rotation_size = 10 * 1024 * 1024,
                    keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
                    keywords::auto_flush = true,
                    keywords::format = log_string
                );

                logging::add_console_log(std::cout, boost::log::keywords::format = log_string);
            }
    };
}

#define LOG_DEBUG   BOOST_LOG_SEV( freeture_logger::get(), trivial::debug)
#define LOG_INFO    BOOST_LOG_SEV( freeture_logger::get(), trivial::info)
#define LOG_WARNING BOOST_LOG_SEV( freeture_logger::get(), trivial::warning)
#define LOG_ERROR   BOOST_LOG_SEV( freeture_logger::get(), trivial::error)
