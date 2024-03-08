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

        std::vector<int>* m_NextSunrise = nullptr;
        std::vector<int>* m_NextSunset = nullptr;

        bool m_Writing = false;
        static NodeExporterMetrics* m_Instance;
        NodeExporterMetrics(std::string);

        std::string m_CompleteDataPath="";
        std::string m_cDate = "";
    public:
        std::string OutputPath = "/freeture/freeture_metric";

        NodeExporterMetrics(NodeExporterMetrics& other) = delete;
        void operator=(const NodeExporterMetrics&) = delete;

        static NodeExporterMetrics& GetInstance(std::string station_code);

        virtual ~NodeExporterMetrics();

        //metrics from AcqThread
        void UpdateMetrics(double, double,double, std::vector<int>*, std::vector<int>*);

        //metrics from DetThread
        void UpdateMetrics(int, double);

        //metrics from AcqThread
        void UpdateMetrics(double, double);

        void UpdateMetrics(std::string, std::string);

        //write metrics to file
        void WriteMetrics();
    };
}
