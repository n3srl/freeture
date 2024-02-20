/**
* \file    main.cpp
* \author  Andrea Novati
* \version 1.0
* \date    03/22/2023
*/
#include <memory>

#include "Freeture.h"
#include "Logger.h"

std::unique_ptr<freeture::Freeture> freeture_instance;

int main(int argc, const char ** argv)
{
    try
    {
        freeture_instance = std::make_unique<freeture::Freeture>(argc,argv);
        freeture_instance->Run();
    }
    catch(...)
    {
        std::cout << "main;" << ">> Generic Error"  << std::endl;
    }

    return 0 ;
}

