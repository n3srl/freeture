/**
* \file    main.cpp
* \author  Andrea Novati
* \version 1.0
* \date    03/22/2023
*/

#include "Freeture.h"

using namespace std;
using namespace freeture;

int main(int argc, const char ** argv)
{
    try
    {
        Freeture* freeture =new Freeture(argc,argv);

        freeture->Run();
    }
    catch(exception& e)
    {
        cout << ">> Error : " << e.what() << endl;
    }
    catch(const char * msg)
    {
        cout << ">> Error : " << msg << endl;
    }

    return 0 ;
}

