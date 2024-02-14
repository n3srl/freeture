/**
* \file    main.cpp
* \author  Andrea Novati
* \version 1.0
* \date    03/22/2023
*/
#include <memory>

#include "Freeture.h"
#include "Logger.h"

using namespace std;
using namespace freeture;

unique_ptr<Freeture> freeture_instance;

int main(int argc, const char ** argv)
{
    Logger& logger = Logger::Get();

    LOG_INFO << "================================================" << endl;
    LOG_INFO << "======        FREETURE - "<< VERSION <<"           ======= " << endl;
    LOG_INFO << "================================================" << endl << endl;

    try
    {
        freeture_instance = make_unique<Freeture>(argc,argv);
        freeture_instance->Run();
    }
    catch(exception& e)
    {
        LOG_ERROR << "main;" << ">> Error : " << e.what() << endl;
    }
    catch(const char * msg)
    {
        LOG_ERROR << "main;" << ">> Error : " << msg << endl;
    }

    return 0 ;
}

