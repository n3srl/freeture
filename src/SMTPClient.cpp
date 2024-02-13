/*
                                SMTPClient.cpp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                       2016-2018 Chiara Marmo
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
*   Last modified:      10/12/2018
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    SMTPClient.cpp
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.3
* \date    10/12/2018
* \brief   SMTP connection and send mails.
*/

#include "SMTPClient.h"
#include "Socket.h"
#include "Base64.h"

using namespace freeture;

bool SMTPClient::checkSMTPAnswer(const std::string & responseWaited, boost::asio::ip::tcp::socket & socket) {

    bool res = true;

    //http://www.boost.org/doc/libs/1_40_0/doc/html/boost_asio/reference/streambuf.html
    boost::asio::streambuf response;
    std::string code;

    // Read data into a streambuf until it contains "\r\n".
    boost::asio::read_until(socket, response, "\r\n");

    {
        std::istream is(&response);
        is >> code;

        if(code != responseWaited){

            std::cerr << "Not correct expecting answer from SMTP server : " << code << std::endl;
            res = false;

        }else{

            //cout << "code : " <<code <<endl;
            LOG_INFO << "code :" << code;

        }

        // Remove characters from response.
        response.consume(response.size());

        // IO control command to get the amount of data that can be read without blocking.
        boost::asio::socket_base::bytes_readable command(true);
        socket.io_control(command);

        while(command.get()) {

            boost::asio::read_until(socket, response, "\r\n");
            socket.io_control(command);

        }

        response.consume( response.size() );

    }

    return res;
}

void SMTPClient::write(std::string data, std::string expectedAnswer, bool checkAnswer, boost::asio::ip::tcp::socket & socket) {

    boost::asio::write(socket, boost::asio::buffer(data));

    if(checkAnswer)
        checkSMTPAnswer(expectedAnswer, socket);

}

bool SMTPClient::getFileContents(const char *filename, std::string &content){

    std::ifstream in(filename, std::ios::in | std::ios::binary);
    std::cout << filename<< std::endl;
    if(in){

        in.seekg(0, std::ios::end);
        content.resize(in.tellg());
        in.seekg(0, std::ios::beg);
        in.read(&content[0], content.size());
        in.close();
        return true;

    }

    return false;

}


std::string SMTPClient::buildMessage( std::string msg, std::vector<std::string> mMailAttachments,
                                 std::vector<std::string> mMailTo,  std::string mMailFrom,  std::string mMailSubject){

    // Final data to send.
    std::string message;

    // In case where mail client doesn't support HTML.
    std::string rawMessage = msg;

    // Used to separate different mail formats.
    std::string section = "08zs01293eraf47a7804dcd17b1e";

    // Message using HTML.
    std::string htmlMessage =    "<html>\
                                <body>\
                                    <p> " + msg + " </p> ";
                 htmlMessage+= "</body>\
                             </html>";

    // Specify the MIME version used.
    message = "Mime-Version: 1.0\r\n";

    // Head of the message starting by the Sender.
    message += "from: no-reply<" + mMailFrom + ">\r\n";

    // Recipients.
    for(int i = 0; i < mMailTo.size(); i++)
        message += "To: <" + mMailTo.at(i) + ">\r\n";

    // Subject.
    message += "subject: " + mMailSubject + "\r\n";

    /*

    MAIL STRUCTURE MODEL :

    multipart/mixed
        multipart/alternative
            text/plain
            multipart/related
                text/html
                image/jpg
        some/thing (disposition:attachment)
        some/thing (disposition:attachment)

    */

    message += "Content-Type: multipart/mixed; boundary=" + section  + "\r\n";
    message += "\r\n";

        message += "Content-Type: multipart/alternative; boundary=" + section + "\r\n";
        message += "\r\n";

            // Raw text.
            message += "\r\n--" + section  + "\r\n";

            message += "Content-type: text/plain; charset=ISO-8859-1\r\n";
            message += "\r\n";
            message += rawMessage;
            message += "\r\n";

            message += "\r\n--" + section  + "\r\n";

            message += "Content-Type: multipart/related; boundary=" + section  + "\r\n";
            message += "\r\n";

                // HTML text.
                message += "--" + section  + "\r\n";
                message += "Content-type: text/html; charset=ISO-8859-1\r\n";
                message += "\r\n";
                message += htmlMessage ;
                message += "\r\n";

                message += "\r\n--" + section  + "\r\n";

        // ATTACHMENTS.

        // .txt attachment.
        /*message += "\r\n--" + section  + "\r\n";

        message += "Content-Type: text/plain; name =\"test.txt\"\r\n";
        message += "Content-Disposition: attachment\r\n";
        message += "filename=\"test.txt\"\r\n";

        message += "this is the attachment text\r\n";*/

        // png attachment.
        //http://dataurl.net/#dataurlmaker

        for(int i=0; i<mMailAttachments.size(); i++){

            message += "\r\n--" + section  + "\r\n";

            std::string s = mMailAttachments.at(i);
            std::string delimiter = "/";

            std::vector<std::string> elements;
            std::string fileName;
            std::string fileExtension;

            size_t pos = 0;
            std::string token;
            while((pos = s.find(delimiter)) != std::string::npos) {

                token = s.substr(0, pos);
                elements.push_back(token);
                //cout << token << endl;
                s.erase(0, pos + delimiter.length());

            }

            elements.push_back(s);

            fileName = elements.back();
            //cout << fileName << endl;

            s = mMailAttachments.at(i);
            delimiter = ".";
            elements.clear();

            pos = 0;
            token="";
            while ((pos = s.find(delimiter)) != std::string::npos) {
                token = s.substr(0, pos);
                elements.push_back(token);
                s.erase(0, pos + delimiter.length());
            }
            elements.push_back(s);

            fileExtension = elements.back();
            //  cout << fileExtension << endl;

            message += "Content-Type: image/" + fileExtension + "; name =\"" + fileName + "\"\r\n";
            message += "Content-Transfer-Encoding: Base64\r\n";
            message += "Content-Disposition: attachment\r\n";
            message += "filename=\"" + fileName + "\"\r\n\n";
            // cout << "getFileContents : " << mMailAttachments.at(i)<< endl;
            std::string img;
            if(!getFileContents(mMailAttachments.at(i).c_str(), img)) {
                std::cout << "Fail to load image to attach to mail message" << std::endl;
                
                LOG_ERROR << "Fail to load image to attach to mail message";
                img = "";
            }
            //  cout << "end getFileContents" << endl;
            message += Base64::encodeBase64(img);

            message += "\r\n";

        }

    // Mail end.
    message += "\r\n--" + section  + "--\r\n";

    return message;

}

void SMTPClient::sendMail(  std::string            server,
                            std::string            login,
                            std::string            password,
                            std::string            from,
                            std::vector<std::string>    to,
                            std::string            subject,
                            std::string            message,
                            std::vector<std::string>    pathAttachments,
                            SmtpSecurity      securityType) {

    try {

        switch(securityType) {

            case NO_SECURITY :

                {

                    Socket socket(server, 25);
                    checkSMTPAnswer("220", *socket.GetSocket());

                    // HELO to SMTP server.
                    LOG_INFO << "HELLO to SMTP server.";
                    write("HELO " + server + "\r\n", "250", true, *socket.GetSocket());

                    // Sender.
                    LOG_INFO << "Write sender.";
                    write("MAIL FROM: <" + from + ">\r\n", "250", true, *socket.GetSocket());

                    // Recipients.
                    LOG_INFO << "Write recipients.";
                    for(int i = 0; i < to.size(); i++)
                    write("RCPT TO: <" + to.at(i) + ">\r\n", "250", true, *socket.GetSocket());

                    // Start to sending data.
                    LOG_INFO << "Write datas.";
                    write("DATA\r\n", "354", true, *socket.GetSocket());

                    // Build message using MIME.
                    LOG_INFO << "Build message using MIME.";
                    std::string data = buildMessage(message, pathAttachments, to, from, subject);

                    // Send data.
                    LOG_INFO << "Send data.";
                    write(data, "", false, *socket.GetSocket());

                    // End of sending data.
                    LOG_INFO << "End of sending data.";
                    write("\r\n.\r\n", "250", true, *socket.GetSocket());

                    // Deconnection.
                    LOG_INFO << "Deconnection.";
                    write("QUIT\r\n", "221", true, *socket.GetSocket());

                    LOG_INFO << "Mail sent.";

                }

                break;

            case USE_SSL :

                {

                    Socket socket(server, 465);

                    static const std::string newline = "\r\n";

                    LOG_INFO << "Initialize SSL connection.";
                    OpenSSL::StaticInitialize sslInitializer;

                    OpenSSL openSSL(socket.GetSocket()->native_handle());
                    LOG_INFO << openSSL.Read(ReceiveFunctor(220));

                    LOG_INFO << std::string("EHLO ") << server;
                    openSSL.Write(std::string("EHLO ") + server + newline);
                    LOG_INFO << openSSL.Read(ReceiveFunctor(250));

                    LOG_INFO << "AUTH LOGIN";
                    openSSL.Write(std::string("AUTH LOGIN") + newline);
                    LOG_INFO << openSSL.Read(ReceiveFunctor(334));

                    LOG_INFO << "Write Login";
                    openSSL.Write(Base64::encodeBase64(login) + newline);
                    LOG_INFO << openSSL.Read(ReceiveFunctor(334));

                    LOG_INFO << "Write password";
                    openSSL.Write(password + newline);
                    LOG_INFO << openSSL.Read(ReceiveFunctor(235));

                    LOG_INFO << "MAIL FROM:<" << from << ">";
                    openSSL.Write(std::string("MAIL FROM:<") + from + ">" + newline);
                    LOG_INFO << openSSL.Read(ReceiveFunctor(250));

                    for(int i = 0; i < to.size(); i++) {

                        LOG_INFO << "RCPT TO:<" << to.at(i) << ">";
                        openSSL.Write(std::string("RCPT TO:<") + to.at(i) + ">" + newline);
                        LOG_INFO << openSSL.Read(ReceiveFunctor(250));

                    }

                    LOG_INFO << "DATA";
                    openSSL.Write(std::string("DATA") + newline);
                    LOG_INFO << openSSL.Read(ReceiveFunctor(354));

                    LOG_INFO << "Build message";
                    std::string m = buildMessage(message, pathAttachments, to, from, subject);
                    openSSL.Write( m + newline + "." + newline);
                    //BOOST_LOG_SEV(logger,normal) << openSSL.Read(ReceiveFunctor(250));

                    LOG_INFO << "QUIT";
                    openSSL.Write(std::string("QUIT: ") + newline);

                    LOG_INFO << "Mail sent.";

                }

                break;

            case USE_TLS :

                break;

        }

    }catch(std::exception& e){

       LOG_ERROR << e.what();

    }catch(const char * msg){

       LOG_ERROR << "Fail to send mail : " << msg;

    }
}
