#pragma once
#include "Commons.h"

#include <mutex>
#include "ILogger.h"
#include "ELogSeverityLevel.h"

namespace freeture
{
    class Logger;

    class Logger_Console : public ILogger
    {
    public:
        Logger_Console( Logger* log):ILogger(log){

        }
        void init() override {

        }
        void attach(LogThread, thread_id_type) override {
        }
        void reset() override {
        }
    };
}
