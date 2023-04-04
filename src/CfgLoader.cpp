/*
                            CfgLoader.cpp

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
*   Last modified:      20/10/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    CfgLoader.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    13/06/2014
* \brief   Methods to fetch parameters from configuration file.
*/

#include "CfgLoader.h"

CfgLoader::CfgLoader(void){}

void CfgLoader::Clear(){

    mData.clear();

}

bool CfgLoader::Load(const std::string& file){

    std::ifstream inFile(file.c_str());

    if (!inFile.good()){
        return false;
    }

    std::string prevKey = "";

    while (inFile.good() && ! inFile.eof()){

        std::string line;
        getline(inFile, line);

        // filter out comments
        if (!line.empty()){

            int pos = line.find('#');

            if (pos != std::string::npos){

                line = line.substr(0, pos);

            }
        }

        // split line into key and value
        if (!line.empty()){

            int pos = line.find('=');

            // "=" not found.
            if (pos != std::string::npos){

                std::string key     = Trim(line.substr(0, pos));
                std::string value   = Trim(line.substr(pos + 1));

                if (!key.empty() && !value.empty()){

                    prevKey   = key;
                    mData[key] = value;

                }

            }else if(line.size() > 1 && !prevKey.empty()){

                mData[prevKey] += Trim(line);

            }
        }
    }

    return true;
}

bool CfgLoader::Contains(const std::string& key) const{

    return mData.find(key) != mData.end();
}

bool CfgLoader::Get(const std::string& key, std::string& value) const{

    std::map<std::string,std::string>::const_iterator iter = mData.find(key);

    if(iter != mData.end()){

        value = iter->second;
        return true;

    }else{

        return false;
    }
}

bool CfgLoader::Get(const std::string& key, int& value) const{

    std::string str;

    if(Get(key, str)){

        value = atoi(str.c_str());
        return true;

    }else{

        return false;
    }
}

bool CfgLoader::Get(const std::string& key, long& value) const{

    std::string str;

    if(Get(key, str)){

        value = atol(str.c_str());
        return true;

    }else{

        return false;
    }
}

bool CfgLoader::Get(const std::string& key, double& value) const{

    std::string str;

    if(Get(key, str)){

        value = atof(str.c_str());
        return true;

    }else{

        return false;
    }
}

bool CfgLoader::Get(const std::string& key, bool& value) const{

    std::string str;

    if(Get(key, str)){

        value = (str == "true");
        return true;

    }else{

        return false;
    }
}

std::string CfgLoader::Trim(const std::string& str){

    int first = str.find_first_not_of(" \t");

    if(first != std::string::npos){

        int last = str.find_last_not_of(" \t");

        return str.substr(first, last - first + 1);

    }else{

        return "";
    }
}
