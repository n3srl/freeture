#pragma once
/*
                                Socket.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau -- GEOPS-UPSUD
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
*   Last modified:      03/03/2015
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    Socket.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    03/03/2015
*/
//header refactoring ok
#include "Commons.h"

#include <boost/asio.hpp>
#include <boost/archive/iterators/ostream_iterator.hpp>

namespace freeture
{


    class Socket {

        boost::asio::io_service mIoService;
        boost::asio::ip::tcp::socket mSocket;

    public:

        /**
        * Constructor.
        *
        * @param sever SMTP server.
        * @param port Connection port.
        */
        Socket(std::string server, uint16_t port) : mSocket(mIoService) {

            boost::asio::ip::tcp::resolver resolver(mIoService);

            boost::asio::ip::tcp::resolver::query query(server, std::to_string(port));

            boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

            boost::asio::connect(mSocket, endpoint_iterator);
        }

        /**
        * Get socket.
        *
        * @return Pointer on the socket.
        */
        boost::asio::ip::tcp::socket* GetSocket() {
            return &mSocket;
        }

    };
}
