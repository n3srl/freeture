#pragma once

#include "config.h"
#include "Constants.h"
#include "PlatformSwitches.h"

#define BOOST_THREAD_PROVIDES_FUTURE

#ifdef LINUX
    #define BOOST_LOG_DYN_LINK 1
#endif

#ifdef WINDOWS
#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#define NOMINMAX 

#ifndef __wtypes_h__
#include <wtypes.h>
#endif

#endif
