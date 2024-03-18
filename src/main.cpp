/**
* \file    main.cpp
* \author  BSIT, Andrea Novati - N3 S.r.l. (Significant Refactoring)
* \version 1.0
* \date    03/22/2023
*/
#include <memory>
#include <iostream>

#include "Freeture.h"

using namespace freeture;
using namespace std;

shared_ptr<Freeture> freeture_instance;

int main(int argc, const char ** argv)
{
    try
    {
        freeture_instance = make_shared<Freeture>(argc,argv);
        freeture_instance->Run();
    }
    catch(exception& e)
    {
        cout << "main;" << e.what()  << endl;
    }
    catch (...)
    {
        cout << "main;" << "GENERIC ERROR";
    }

    return 0 ;
}

