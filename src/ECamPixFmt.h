#pragma once
/*
                            ECamPixFmt.h

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*
*   This file is part of:   freeture
*
*   Copyright:      (C) 2014-2015 Yoan Audureau
*                               GEOPS-UPSUD-CNRS
*                   (C) 2020-2024 Andrea Novati - N3 S.r.l.
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
*   Last modified:      01/12/2014
*
*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
namespace freeture
{
    enum class CamPixFmt {

        MONO8,  // = Y800 = GREY
        MONO12, // Msb aligned : XXXXnnnnnnnnnnnn (X = unused)
        GREY,
        Y800,
        YUYV,
        UYVY,
        RGB565,
        MONO16,
        BGR3,
        RGB3,
        UNDEFINED
    };
}
