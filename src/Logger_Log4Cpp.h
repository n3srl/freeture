#pragma once
/**
* \file    Logger.h
* \author   Andrea Novati - N3 s.r.l.
* \version 1.0
* \date    11/02/2024
*/
#include "Commons.h"

#include <mutex>
#include <unordered_map>
#include <log4cpp/Category.hh>

#include "ILogger.h"
#include "ELogSeverityLevel.h"


namespace freeture
{
  
    class Logger_Log4Cpp: public ILogger
    {

    public:
        Logger_Log4Cpp(Logger*);

        void init() override;
        void attach(LogThread, thread_id_type) override;
        void reset() override;


        void logDebug(const std::string& message);
        void logInfo(const std::string& message);
        void logWarning(const std::string& message);
        void logError(const std::string& message);
        void logFatal(const std::string& message);

        void setLogThread(LogThread, thread_id_type, bool reset = false);

    private:
        log4cpp::Priority::PriorityLevel logLevel;
        log4cpp::Category& getCategory();
        std::mutex mutex;
        std::unordered_map<std::thread::id, log4cpp::Category*> categories;
    };
}
