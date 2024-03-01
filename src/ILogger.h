#pragma once
#include <thread>
#include <iostream>
#include <sstream>
#include <mutex>
#include <memory>
#include <string>
#include <chrono>
#include <iomanip>

#include "ELogThread.h"
#include "ELogSeverityLevel.h"

using thread_id_type = std::thread::id;


namespace freeture
{
    class Logger;

    class ILogger
    {
    protected:
        std::ostringstream m_Stream;
        std::mutex m_StreamMutex;
        LogSeverityLevel m_LogLevel;

    public:
        std::shared_ptr<Logger> logger;

        ILogger(std::shared_ptr<Logger> log) :
            logger(log),
            m_LogLevel(LogSeverityLevel::normal),
            m_Stream()
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

        virtual void apply() = 0;

        virtual void flush(std::string) = 0;


        template<typename T>
        ILogger& operator<<(const T& value) {
            std::lock_guard<std::mutex> lock(m_StreamMutex); // Ensure thread safety
            m_Stream << value;
            return *this;
        }

        // Specialize for manipulators (e.g., std::endl)
        ILogger& operator<<(std::ostream& (*pf)(std::ostream&)) {
            std::lock_guard<std::mutex> lock(m_StreamMutex); // Ensure thread safety
            m_Stream << pf;
            if (pf == static_cast<std::ostream & (*)(std::ostream&)>(std::flush) ||
                pf == static_cast<std::ostream & (*)(std::ostream&)>(std::endl)) {
                flush(); // Auto-flush on endl or flush manipulator
            }
            return *this;
        }

        ILogger& operator<<(const char* text) {
            std::lock_guard<std::mutex> lock(m_StreamMutex); // Ensure thread safety
            m_Stream << std::string(text);
            return *this;
        }


        ILogger& operator<<(const std::string& text) {
            std::lock_guard<std::mutex> lock(m_StreamMutex); // Ensure thread safety
            m_Stream << text;
            return *this;
        }

        void level(LogSeverityLevel level) {
            m_LogLevel = level;
        }


        private:
            void console_log(std::string message) {
                // Get current time point
                auto now = std::chrono::system_clock::now();
                // Convert time point to system time_t to make it compatible with time functions
                auto now_c = std::chrono::system_clock::to_time_t(now);

                // Print the timestamp. Adjust the format as needed
                std::cout << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X; ");

                switch (m_LogLevel) {
                case LogSeverityLevel::critical:
                    std::cout << "CRITICAL; ";
                    break;
                case LogSeverityLevel::fail:
                    std::cout << "FAIL; ";
                    break;
                case LogSeverityLevel::normal:
                    std::cout << "NORMAL; ";
                    break;
                case LogSeverityLevel::notification:
                    std::cout << "NOTIFICATION; ";
                    break;
                case LogSeverityLevel::warning:
                    std::cout << "WARNING; ";
                    break;
                }

                std::cout << message;
            }

            /// <summary>
            /// Call to flush stream
            /// </summary>
            void flush()
            {
                flush(m_Stream.str());
                console_log(m_Stream.str());

                m_Stream.str(""); // Clear the stream buffer
                m_Stream.clear(); // Reset any error flags
            }

    };
}
