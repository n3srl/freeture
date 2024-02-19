/*
                                OpenSSL.cpp

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
* \file    OpenSSL.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    30/05/2015
*/

#include "OpenSSL.h"

using namespace freeture;

OpenSSL::OpenSSL(int socket): ctx_(nullptr, SSL_CTX_free), ssl_(nullptr, SSL_free) {

    char errorBuf[errorBufSize];

    ctx_ = decltype(ctx_ ) (SSL_CTX_new(SSLv23_client_method()), SSL_CTX_free);
    if(nullptr == ctx_) {
        LOG_ERROR << "SSL_CTX_new failed : " << ERR_error_string(ERR_get_error(), errorBuf);
        throw "SSL_CTX_new failed.";
        //throw runtime_error(ERR_error_string(ERR_get_error(), errorBuf));
    }

    ssl_ = decltype(ssl_ ) (SSL_new(ctx_.get()), SSL_free);
    if(nullptr == ssl_) {
        LOG_ERROR << "SSL_new failed : " << ERR_error_string(ERR_get_error(), errorBuf);
        throw "SSL_new failed.";
        //throw runtime_error(ERR_error_string(ERR_get_error(), errorBuf));
    }

    const int rstSetFd = SSL_set_fd(ssl_.get(), socket);
    if(0 == rstSetFd) {
        LOG_ERROR << "SSL_set_fd failed : " << ERR_error_string(ERR_get_error(), errorBuf);
        throw "SSL_set_fd failed.";
        //throw runtime_error(ERR_error_string(ERR_get_error(), errorBuf));
    }

    const int rstConnect = SSL_connect(ssl_.get());
    if(0 == rstConnect) {
        LOG_ERROR  << "Handshake failed. ";
        throw "Handshake failed. ";
        //throw runtime_error("handshake failed.");
    }else if(0> rstConnect) {
        LOG_ERROR << "Handshake and shutdown failed. ";
        throw "Handshake and shutdown failed. ";
        //throw runtime_error("handshake and shutdown failed.");
    }
}

OpenSSL::~OpenSSL() {

    int rstShutdown = SSL_shutdown(ssl_.get());
    if(0==rstShutdown)
        rstShutdown = SSL_shutdown(ssl_.get());
    else if(-1 == rstShutdown && SSL_RECEIVED_SHUTDOWN != SSL_get_shutdown(ssl_.get())) {
        LOG_ERROR << "Shutdown failed.";
        //throw "Shutdown failed.";
        //throw runtime_error("shutdown failed.");
    }

}

void OpenSSL::Write(const std::string &msg) {

    const int rstWrite = SSL_write(ssl_.get(), msg.c_str(), msg.length());
    if(0 == rstWrite) {
        LOG_ERROR << "Socket write failed due to lose connection.";
        throw "Socket write failed due to lose connection.";
        //throw runtime_error("socket write failed due to lose connection.");
    }else if(0> rstWrite) {
        LOG_ERROR << "Socket write failed due to unknown reason.";
        throw "Socket write failed due to unknown reason.";
        //throw runtime_error("socket write failed due to unknown reason.");
    }
}
