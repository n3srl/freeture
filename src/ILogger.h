#pragma once
#include <thread>
#include <iostream>
#include <sstream>
#include <mutex>
#include <string>

#include "ELogThread.h"
#include "ELogSeverityLevel.h"

using thread_id_type = std::thread::id;



namespace freeture
{
    class Logger;

    class ILogger
    {
    private:
        std::ostringstream m_Stream;
        std::mutex mtx;
        LogSeverityLevel m_LogLevel;

    public:
        Logger* logger;

        ILogger(Logger* log) :
            logger(log),
            m_LogLevel(LogSeverityLevel::normal)
        {
        }

        std::ostringstream& stream() { return m_Stream; }

        /// <summary>
        /// Attach a thread to loggers, concrete implementation, called by setLogThread
        /// </summary>
        /// <param name=""></param>
        /// <param name=""></param>
        virtual void attach(LogThread, thread_id_type) = 0;

        /// <summary>
        /// init concrete logger
        /// </summary>
        virtual void init() = 0;

        /// <summary>
        /// reset the concrete logger called by setLogThread 
        /// </summary>
        virtual void reset() = 0;


        void log(const std::string& message) {
            std::lock_guard<std::mutex> lock(mtx); // Ensure thread safety
            m_Stream << message;
        }

        virtual void flush() {
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

        template<typename T>
        ILogger& operator<<(const T& value) {
            std::lock_guard<std::mutex> lock(mtx); // Ensure thread safety
            m_Stream << value;
            return *this;
        }


        // Specialize for manipulators (e.g., std::endl)
        ILogger& operator<<(std::ostream& (*pf)(std::ostream&)) {
            std::lock_guard<std::mutex> lock(mtx); // Ensure thread safety
            m_Stream << pf;
            if (pf == static_cast<std::ostream & (*)(std::ostream&)>(std::flush) ||
                pf == static_cast<std::ostream & (*)(std::ostream&)>(std::endl)) {
                flush(); // Auto-flush on endl or flush manipulator
            }
            return *this;
        }

        ILogger& ILogger::operator<<(const char* text) {
            std::lock_guard<std::mutex> lock(mtx); // Ensure thread safety
            m_Stream << std::string(text);
            return *this;
        }


        ILogger& ILogger::operator<<(const std::string& text) {
            std::lock_guard<std::mutex> lock(mtx); // Ensure thread safety
            m_Stream << text;
            return *this;
        }

        void level(LogSeverityLevel level) {
            m_LogLevel = level;
        }

    };
}
