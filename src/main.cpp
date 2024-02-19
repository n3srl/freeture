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
    try
    {
        freeture_instance = make_unique<Freeture>(argc,argv);
        freeture_instance->Run();
    }
    catch(...)
    {
        cout << "main;" << ">> Generic Error"  << endl;
    }

    return 0 ;
}

