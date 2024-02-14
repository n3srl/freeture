#pragma once
#include <string>

#include "Constants.h"

struct FreetureSettings {
    int         mode = -1;
    int         executionTime = 0;

    std::string      savePath = "./";
    int         acqFormat = 0;
    int         acqWidth = 0;
    int         acqHeight = 0;
    int         startx = 0;
    int         starty = 0;
    int         gain = 0;
    double      exp = 0;
    int         devID = 0;
    std::string      fileName = "snap";
    bool        listFormats = false;
    bool        display = false;
    bool        bmp = false;
    bool        fits = false;
    bool        sendbymail = false;
    std::string configurationFilePath = std::string(CFG_PATH) + "configuration.cfg";
};