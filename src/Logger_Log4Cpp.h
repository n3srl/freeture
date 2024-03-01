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
#include <log4cpp/Priority.hh>

#include "ILogger.h"
#include "ELogSeverityLevel.h"

#ifdef LINUX
using namespace log4cpp_GenICam;
#endif

#ifndef LINUX
using namespace log4cpp;
#endif

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

        Category* m_AcqThreadCategory;
        Category* m_StackThreadCategory;
        Category* m_FreetureCategory;
        Category* m_DetThreadCategory;

        void fetchCategories();
        Category& getCategory();
        void updateAppenderConfiguration(std::string);
        void setRootLogLevel(int);
        void logDebug(const std::string&);
        void logNotification(const std::string&);
        void logWarning(const std::string&);
        void logFail(const std::string&);
        void logCritical(const std::string&);
        void logNormal(const std::string&);

        void preprocessAndLoadConfiguration(const std::string&);
    };
}
