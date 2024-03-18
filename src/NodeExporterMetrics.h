#pragma once
#include "Commons.h"

#include <string>
#include <vector>

namespace freeture
{
    class NodeExporterMetrics
    {
    private:
        int m_DetNbDetection;
        double m_DetTime;

        double m_AcqFPS;
        double m_AcqTime;
        double m_Temperature;

        long m_StartSunriseTime;
        long m_StopSunriseTime;
        long m_StartSunsetTime;
        long m_StopSunsetTime;

        bool m_Writing = false;
        static NodeExporterMetrics* m_Instance;
        NodeExporterMetrics(std::string);

        std::string m_CompleteDataPath="";
        std::string m_cDate = "";
        std::string m_StationName = "";
    public:
        std::string OutputPath = DEFAULT_METRICS_OUTPUT_FILE;

        NodeExporterMetrics(NodeExporterMetrics& other) = delete;
        void operator=(const NodeExporterMetrics&) = delete;

        static NodeExporterMetrics& GetInstance(std::string station_code);

        virtual ~NodeExporterMetrics();

        //metrics from AcqThread
        void UpdateMetrics(long, long, long, long);

        //metrics from DetThread
        void UpdateMetrics(int, double);

        //metrics from AcqThread
        void UpdateMetrics(double,double, double);

        void UpdateMetrics(std::string, std::string);

        //write metrics to file
        void WriteMetrics();
    };
}
