/*
                                Fits.h

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
*   Last modified:      21/10/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
* \file    Fits.h
* \author  Yoan Audureau -- GEOPS-UPSUD
* \version 1.0
* \date    04/12/2014
*/

#pragma once

#include "SParam.h"
#include "config.h"

#include <boost/filesystem.hpp>

using namespace boost::filesystem;


class Fits {

     public :

        std::string  STATION;

        // Fits Header Keywords Name.

        std::string  kFILENAME;
        std::string  kDATE;
        std::string  kDATEOBS;
        std::string  kOBSMODE;
        double  kELAPTIME;
        double  kEXPOSURE;
        double  kONTIME;
        std::string  kFILTER;
        std::string  kTELESCOP;
        std::string  kOBSERVER;
        std::string  kINSTRUME;
        std::string  kCAMERA;
        double  kFOCAL;
        double  kAPERTURE;
        double  kSITELONG;
        double  kSITELAT;
        double  kSITEELEV;
        double  kXPIXEL;
        double  kYPIXEL;
        int     kGAINDB;
        double  kSATURATE;
        std::string  kPROGRAM;
        std::string  kCREATOR;
        double  kBZERO;
        double  kBSCALE;
        std::string  kRADESYS;
        std::string  kTIMESYS;
        double  kEQUINOX;
        std::string  kCTYPE1;
        std::string  kCTYPE2;
        std::string  kCTYPE3;
        std::string  kTIMEUNIT;
        double  kCD1_1;
        double  kCD1_2;
        double  kCD2_1;
        double  kCD2_2;
        double  kCD3_3;
        double  kCD1_3;
        double  kCD2_3;
        double  kCD3_1;
        double  kCD3_2;
        int     kCRPIX1;
        int     kCRPIX2;
        int     kCRPIX3;
        double  kCRVAL1;
        double  kCRVAL2;
        double  kK1;
        double  kK2;
        std::string  kCOMMENT;

        // Fits Header Keywords Comments.

        std::string  cFILENAME;
        std::string  cDATE;
        std::string  cDATEOBS;
        std::string  cOBSMODE;
        std::string  cELAPTIME;
        std::string  cEXPOSURE;
        std::string  cONTIME;
        std::string  cFILTER;
        std::string  cTELESCOP;
        std::string  cOBSERVER;
        std::string  cINSTRUME;
        std::string  cCAMERA;
        std::string  cFOCAL;
        std::string  cAPERTURE;
        std::string  cSITELONG;
        std::string  cSITELAT;
        std::string  cSITEELEV;
        std::string  cXPIXEL;
        std::string  cYPIXEL;
        std::string  cGAINDB;
        std::string  cSATURATE;
        std::string  cPROGRAM;
        std::string  cCREATOR;
        std::string  cBZERO;
        std::string  cBSCALE;
        std::string  cRADESYS;
        std::string  cTIMESYS;
        std::string  cEQUINOX;
        std::string  cCTYPE1;
        std::string  cCTYPE2;
        std::string  cCTYPE3;
        std::string  cTIMEUNIT;
        std::string  cCD1_1;
        std::string  cCD1_2;
        std::string  cCD2_1;
        std::string  cCD2_2;
        std::string  cCD3_3;
        std::string  cCD1_3;
        std::string  cCD2_3;
        std::string  cCD3_1;
        std::string  cCD3_2;
        std::string  cCRPIX1;
        std::string  cCRPIX2;
        std::string  cCRPIX3;
        std::string  cCRVAL1;
        std::string  cCRVAL2;
        std::string  cK1;
        std::string  cK2;
        std::string  cCOMMENT;

    public :

        Fits();

        ~Fits();

        void loadKeys(fitskeysParam fkp, stationParam sp);
};

