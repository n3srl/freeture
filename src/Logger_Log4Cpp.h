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
        Logger_Log4Cpp(std::shared_ptr<Logger>);

        void init() override;
        void attach(LogThread, thread_id_type) override;
        void reset() override;
        void flush(std::string message) override;
        void apply() override;
    private:
        bool m_Init = false;
        std::mutex m_RecordMutex;

        log4cpp::Category* m_AcqThreadCategory;
        log4cpp::Category* m_StackThreadCategory;
        log4cpp::Category* m_FreetureCategory;
        log4cpp::Category* m_DetThreadCategory;

        void fetchCategories();
        log4cpp::Category& getCategory();
        void updateAppenderConfiguration(std::string);
        void setRootLogLevel(log4cpp::Priority::Value);
        void logDebug(const std::string&);
        void logNotification(const std::string&);
        void logWarning(const std::string&);
        void logFail(const std::string&);
        void logCritical(const std::string&);
        void logNormal(const std::string&);

        void preprocessAndLoadConfiguration(const std::string&);
    };
}
