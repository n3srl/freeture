/*
                                Logger.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                               GEOPS-UPSUD-CNRS
*
*   License:        GNU General Public License
*
*   FreeTure is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*   FreeTure is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*   You should have received a copy of the GNU General Public License
*   along with FreeTure. If not, see <http://www.gnu.org/licenses/>.
*
*   Last modified:      20/07/2015
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    Logger.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/06/2014
*/

#pragma once

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <ostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <string>
#include <numeric>
#include <iterator>
#include <vector>
#include <filesystem>

#include <boost/log/common.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/core.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/gregorian/greg_date.hpp>
#include <boost/bind.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include "TimeDate.h"



namespace fs        = boost::filesystem;

namespace freeture
{


    enum class Level
    {
        WARNING,
        ERROR,
        DEBUG,
        INFO
    };

    class Logger
    {

        private :

            std::string mLogPath;
            int mTimeLimit;
            int mSizeLimit;
            std::vector<std::string> mLogFiles;
            std::vector<int> mRefDate;

            void  getFoldersize(std::string rootFolder,unsigned long long & f_size)
            {
                fs::path folderPath(rootFolder);

                if (fs::exists(folderPath)) {

                    fs::directory_iterator end_itr;

                    for (fs::directory_iterator dirIte(rootFolder); dirIte != end_itr; ++dirIte ) {

                        fs::path filePath(complete (dirIte->path(), folderPath));

                        try{

                            if (!is_directory(dirIte->status()) ){
                                f_size = f_size + file_size(filePath);
                            }else{
                                getFoldersize(filePath.string(),f_size);
                            }

                        }catch(std::exception& e){  std::cout << e.what() << std::endl; }
                    }
                }
            }

        public :

            static void LogLevel(freeture::Level level)
            {
                switch (level)
                {
                    case Level::DEBUG:   std::cout<< "DEBUG: "; break;
                    case Level::ERROR:   std::cout<< "ERROR: "; break;
                    case Level::INFO:    std::cout<< "INFO: "; break;
                    case Level::WARNING: std::cout<< "WARNING: "; break;
                }
            }

            /**
            * Constructor.
            *
            */
            Logger(std::string logPath, int timeLimit, int sizeLimit, std::vector<std::string> logFiles):
            mLogPath(logPath), mTimeLimit(timeLimit), mSizeLimit(sizeLimit), mLogFiles(logFiles) {

                mRefDate = TimeDate::splitStringToInt(TimeDate::localDateTime(microsec_clock::universal_time(),"%Y:%m:%d:%H:%M:%S"));

            }

            void monitorLog() {

                std::vector<int> currDate = TimeDate::splitStringToInt(TimeDate::localDateTime(microsec_clock::universal_time(),"%Y:%m:%d:%H:%M:%S"));

                //cout << "REFDATE : " << mRefDate.at(0) << mRefDate.at(1) << mRefDate.at(2) << endl;
                //cout << "CURDATE : " << currDate.at(0) << currDate.at(1) << currDate.at(2) << endl;

                // Create log date directories when date changes.
                if(mRefDate.at(0) != currDate.at(0) || mRefDate.at(1) != currDate.at(1) || mRefDate.at(2) != currDate.at(2)) {

                    std::string rDate = Conversion::numbering(2, mRefDate.at(0)) + Conversion::intToString(mRefDate.at(0)) + Conversion::numbering(2, mRefDate.at(1)) + Conversion::intToString(mRefDate.at(1)) + Conversion::numbering(2, mRefDate.at(2)) + Conversion::intToString(mRefDate.at(2));
                    //cout << rDate << endl;
                    if(fs::create_directory(fs::path(mLogPath + "/LOG_" + rDate)) || fs::exists(fs::path(mLogPath + "/LOG_" + rDate))) {

                        //cout << mLogPath << "/LOG_" << rDate << " created." << endl;

                        for(int i = 0; i < mLogFiles.size(); i++) {

                            try {

                                rename(fs::path(mLogPath + "/" + mLogFiles.at(i)), fs::path(mLogPath + "/LOG_" + rDate + "/" + mLogFiles.at(i)));
                                //cout << "RENAME : " << mLogPath << "/" << mLogFiles.at(i) << " TO " << mLogPath << "/LOG_" << rDate << "/" + mLogFiles.at(i) << endl;

                            }catch(boost::filesystem::filesystem_error e) {

                                std::cout <<"filesystem error" << std::endl;

                            }
                        }

                        // Clean logs directories

                        boost::posix_time::ptime t1(boost::posix_time::from_iso_string(rDate + "T000000"));
                        int dirNb = 0;
                        std::vector<std::string> dirToRemove;
                        fs::path pDir(mLogPath);
                        //cout << ">> LOOP DIR :  "<< endl;
                        for(fs::directory_iterator file(pDir);file!= fs::directory_iterator(); ++file) {

                            fs::path curr(file->path());
                            //cout << "-> " << curr << endl;

                            if(fs::is_directory(curr)) {

                                std::string dirName = curr.filename().string();

                                std::vector<std::string> output;
                                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                                boost::char_separator<char> sep("_");
                                tokenizer tokens(dirName, sep);

                                for (tokenizer::iterator tok_iter = tokens.begin();tok_iter != tokens.end(); ++tok_iter) {
                                    output.push_back(*tok_iter);
                                }

                                if(output.size() == 2 && output.back().size() == 8) {

                                    boost::posix_time::ptime t2(boost::posix_time::from_iso_string(output.back() + "T000000"));

                                    boost::posix_time::time_duration td = t1 - t2;
                                    long secTime = td.total_seconds();
                                    //cout << secTime << endl;

                                    if(abs(secTime) > mTimeLimit * 24 * 3600) {
                                        dirToRemove.push_back(curr.string());
                                    }

                                }else{
                                    dirToRemove.push_back(curr.string());
                                    //cout << "remove : " << curr.string() << endl;
                                }
                            }
                        }

                        //cout << ">> SIZE dirToRemove : " << dirToRemove.size() << endl;

                        for(int i = 0; i < dirToRemove.size(); i++) {

                            //cout << ">> REMOVE : " << dirToRemove.at(i) << endl;
                            boost::filesystem::remove_all(fs::path(dirToRemove.at(i)));

                        }

                        mRefDate = currDate;

                    }/*else
                        cout << "DIR not exists" << endl;*/

                }

                // Check log size.
                unsigned long long logSize = 0;
                getFoldersize(mLogPath, logSize);
                //cout << "LOG SIZE : " <<  logSize << endl;
                if((logSize/1024.0)/1024.0 > mSizeLimit) {
                    boost::filesystem::path path_to_remove(mLogPath);
                    for (boost::filesystem::directory_iterator end_dir_it, it(path_to_remove); it!=end_dir_it; ++it) {
                        boost::filesystem::remove_all(it->path());
                    }
                }
            }




    };


    template<typename T>
    void Log(T &&t)
    {
        std::cout << t << std::endl;
    }

    template<typename T>
    void LogError(T &&t)
    {
        freeture::Logger::LogLevel(Level::ERROR);
        std::cout << t << std::endl;
    }

    template<typename T>
    void LogInfo(T &&t)
    {
        freeture::Logger::LogLevel(Level::INFO);
        std::cout << t << std::endl;
    }

    template<typename T>
    void LogWarning(T &&t)
    {
        freeture::Logger::LogLevel(Level::WARNING);
        std::cout << t << std::endl;
    }

    template<typename T>
    void LogDebug(T &&t)
    {
        freeture::Logger::LogLevel(Level::DEBUG);
        std::cout << t << std::endl;
    }

    template<typename Head, typename... Tail>
    void LogInfo(Head &&head, Tail&&... tail)
    {
        freeture::Logger::LogLevel(Level::INFO);
        std::cout << head;
        std::cout << " ";
        freeture::Log(std::forward<Tail>(tail)...);
    }

    template<typename Head, typename... Tail>
    void Log(Head &&head, Tail&&... tail)
    {
        std::cout << head;
        std::cout << " ";
        freeture::Log(std::forward<Tail>(tail)...);
    }


    template<typename Head, typename... Tail>
    void LogError(Head &&head, Tail&&... tail)
    {
        Logger::LogLevel(Level::ERROR);
        std::cout << head;
        std::cout << " ";
        freeture::Log(std::forward<Tail>(tail)...);
    }


    template<typename Head, typename... Tail>
    void LogDebug(Head &&head, Tail&&... tail)
    {
        Logger::LogLevel(Level::DEBUG);
        std::cout << head;
        std::cout << " ";
        freeture::Log(std::forward<Tail>(tail)...);
    }

    template<typename Head, typename... Tail>
    void LogWarning(Head &&head, Tail&&... tail)
    {
        Logger::LogLevel(Level::WARNING);
        std::cout << head;
        std::cout << " ";
        freeture::Log(std::forward<Tail>(tail)...);
    }

}
