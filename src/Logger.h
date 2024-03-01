#pragma once
/**
* \file    Logger.h
* \author   Andrea Novati - N3 s.r.l.
* \version 1.0
* \date    11/02/2024
*/
#include "Commons.h"

#include "ILogger.h"

#include <thread>
#include <string>
#include <memory>
#include <mutex>
#include <map>

#include "ELogSeverityLevel.h"
#include "ELogThread.h"

using thread_id_type = std::thread::id;

namespace freeture
{
    class Logger
    {
        // Deleted copy constructor and assignment operator
        Logger(const Logger&) = delete;
        Logger& operator=(const Logger&) = delete;

    public:

        //static std::shared_ptr<Logger> Get(std::string log_path_folder, int log_archive_days, int log_archive_limit_mb, LogSeverityLevel log_severity);
        ~Logger();

        static std::shared_ptr<Logger> m_LoggerInstance;
        static std::mutex m_LoggerInstanceMutex;
        static std::shared_ptr<Logger> Get(LogThread, thread_id_type, std::string, int, int, LogSeverityLevel);
        static std::shared_ptr<Logger> Get();

        
        /// <summary>
        /// Get the concrete logger
        /// </summary>
        /// <returns></returns>
        static ILogger& GetLogger(LogSeverityLevel);

        /// <summary>
        /// Get the thread id for the attached thread
        /// </summary>
        /// <param name="thread"></param>
        /// <returns></returns>
        std::string getThreadId(LogThread thread);

        /// <summary>
        /// Check if thread is attached
        /// </summary>
        /// <param name="thread"></param>
        /// <returns></returns>
        bool hasThreadId(LogThread thread);
        
        LogThread getLogThread(thread_id_type);
        /// <summary>
        /// return current thread id
        /// </summary>
        /// <returns></returns>
        static std::string getThreadId();

        /// <summary>
        /// Public function for "attach" a thread to this logger
        /// </summary>
        /// <param name=""></param>
        /// <param name=""></param>
        /// <param name="reset"></param>
        void setLogThread(LogThread, thread_id_type, bool reset = false);

        LogSeverityLevel getLogSeverityLevel();

        /// <summary>
        /// Configuration parameter for archive days
        /// </summary>
        int getLogArchiveDays();

        /// <summary>
        /// Configuration parameter for Archive size in Megabytes 
        /// </summary>
        int getLogArchiveLimitMB();

        /// <summary>
        /// Configuration parameter for Log Path folder
        /// </summary>
        std::string getLogPathFolder();

    protected:
      /// <summary>
      /// Configuration parameter for Log severity 
      /// </summary>
      LogSeverityLevel m_LogSeverityLevel;


      /// <summary>
      /// Configuration parameter for archive days
      /// </summary>
      int m_LogArchiveDays;

      /// <summary>
      /// Configuration parameter for Archive size in Megabytes 
      /// </summary>
      int m_LogArchiveLimitMB;


      /// <summary>
      /// Configuration parameter for Log Path folder
      /// </summary>
      std::string m_LogPathFolder;

    private:
        /// <summary>
        /// ctor
        /// </summary>
        /// <param name=""></param>
        /// <param name=""></param>
        /// <param name=""></param>
        /// <param name=""></param>
        /// <param name=""></param>
        /// <param name=""></param>
        Logger(LogThread, thread_id_type, std::string, int, int, LogSeverityLevel);

        /// <summary>
        /// Concrete logger
        /// </summary>
        static std::shared_ptr <ILogger> m_Instance;
     
        /// <summary>
        /// LogThread - thread id map
        /// </summary>
        std::map<LogThread, std::string> m_LogMap;
    };
}



#if (USE_BOOST_LOG)
#define LOG_DEBUG       BOOST_LOG_CHANNEL_SEV( Logger_Boost::get(),Logger::getThreadId(), trivial::debug)
#define LOG_INFO        BOOST_LOG_CHANNEL_SEV( Logger_Boost::get(),Logger::getThreadId(), trivial::info)
#define LOG_WARNING     BOOST_LOG_CHANNEL_SEV( Logger_Boost::get(),Logger::getThreadId(), trivial::warning)
#define LOG_ERROR       BOOST_LOG_CHANNEL_SEV( Logger_Boost::get(),Logger::getThreadId(), trivial::error)
#define LOG_FATAL       BOOST_LOG_CHANNEL_SEV( Logger_Boost::get(),Logger::getThreadId(), trivial::fatal)
#endif

#if (!USE_BOOST_LOG)
#define LOG_DEBUG       freeture::Logger::GetLogger(freeture::LogSeverityLevel::normal)
#define LOG_INFO        freeture::Logger::GetLogger(freeture::LogSeverityLevel::notification)
#define LOG_WARNING     freeture::Logger::GetLogger(freeture::LogSeverityLevel::warning)
#define LOG_ERROR       freeture::Logger::GetLogger(freeture::LogSeverityLevel::fail)
#define LOG_FATAL       freeture::Logger::GetLogger(freeture::LogSeverityLevel::critical)
#endif