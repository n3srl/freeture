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
        Logger_Console(std::shared_ptr<Logger> log):ILogger(log){

        }
        void init() override {

        }
        void attach(LogThread, thread_id_type) override {
        }

        void reset() override {
        }

        void flush(std::string message) override {
            if (m_Stream.str().empty()) return; // Don't flush empty streams
            // Here you would typically decide where to output based on logLevel,
            // For simplicity, we're just printing to std::cout

            switch (m_LogLevel) {
            case LogSeverityLevel::normal: {
                std::cout << "DEBUG;";
                break;
            }
            case LogSeverityLevel::notification: {
                std::cout << "INFO;";
                break;
            }
            case LogSeverityLevel::fail: {
                std::cout << "ERROR;";
                break;
            }
            case LogSeverityLevel::critical: {
                std::cout << "CRITICAL ERROR;";
                break;
            }
            case LogSeverityLevel::warning: {
                std::cout << "WARNING;";
                break;
            }
            };

            std::cout << m_Stream.str() << std::endl;

            m_Stream.str(""); // Clear the stream buffer
            m_Stream.clear(); // Reset any error flags
        }

    };
}
