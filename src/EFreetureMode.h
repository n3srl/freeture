#pragma once
namespace freeture {
    enum class FreetureMode
    {
        TEST_CONFIGURATION,
        PRINT_HELP,
        PRINT_VERSION,
        CONTINUOUS_ACQUISITION,
        METEOR_DETECTION,
        SINGLE_ACQUISITION,
        CLEAN_LOGS,
        LIST_DEVICES,
        LIST_FORMATS,
        PRINT_TEST_SCHEDULE,
        UNKNOWN
    };
}