/**
* \file    main.cpp
* \author  Andrea Novati
* \version 1.0
* \date    03/22/2023
*/

#include "Freeture.h"
#include "CameraDeviceManager.h"


using namespace std;
using namespace freeture;




int main(int argc, const char ** argv)
{
    std::cout << "================================================" << endl;
    std::cout << "======        FREETURE 13 - V1.0.1       =======" << endl;
    std::cout << "================================================" << endl << endl;
    try
    {
        
        CameraDeviceManager& manager = CameraDeviceManager::Get();
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

