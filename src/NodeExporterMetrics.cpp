//Write node exporter metrics file for freeture metrics.
#include <chrono>
#include <fstream>
#include <string>
#include <iostream>

#include "NodeExporterMetrics.h"
#include "Base64.h"

using namespace std;
using namespace freeture;

NodeExporterMetrics* NodeExporterMetrics::m_Instance = nullptr;

NodeExporterMetrics& NodeExporterMetrics::GetInstance(std::string station_code)
{
    if (m_Instance == nullptr)
        m_Instance = new NodeExporterMetrics(station_code);

    return *m_Instance;
}

NodeExporterMetrics::~NodeExporterMetrics()
{
    if (m_Instance != nullptr)
        delete m_Instance;
}

void NodeExporterMetrics::UpdateMetrics(double AcqFPS, double AcqTime, double Temperature)
{
    m_AcqTime = AcqTime;
    m_AcqFPS = AcqFPS;
    m_Temperature = Temperature;
}

void NodeExporterMetrics::UpdateMetrics(int DetNbDetection, double DetTime)
{
    m_DetNbDetection = DetNbDetection;
    m_DetTime = DetTime;
}

void NodeExporterMetrics::UpdateMetrics(long mStartSunriseTime, long mStopSunriseTime, long mStartSunsetTime, long mStopSunsetTime)
{
    m_StartSunriseTime = mStartSunriseTime;
    m_StopSunriseTime  = mStopSunriseTime;
    m_StartSunsetTime  = mStartSunsetTime;
    m_StopSunsetTime   = mStopSunsetTime;
}

void NodeExporterMetrics::UpdateMetrics(std::string CompleteDataPath , std::string cDate)
{
    m_CompleteDataPath = CompleteDataPath;
    m_cDate = cDate;
}

void NodeExporterMetrics::WriteMetrics()
{
        try
        {
            //multi thread mutex
            if (!m_Writing)
            {
               m_Writing = true;
               chrono::milliseconds timestamp = chrono::duration_cast< chrono::milliseconds >(chrono::system_clock::now().time_since_epoch());
               std::ofstream out(OutputPath);

               out << "freeture_acq_thread_metric_ts{description=\"Metric acquisition timestamp\"}" << " " << timestamp.count() << endl;

               if (!isnan(m_AcqTime))
                   out << "freeture_acq_thread_time{description=\"Last camera acquisition time [ms]\"} "<< m_AcqTime << endl;

               if (!isnan(m_AcqFPS))
                   out << "freeture_acq_thread_fps{description=\"Camera acquisition thread frames per seconds (floating average on camera buffer size) [Hz]\"} " << m_AcqFPS << endl;
              
               if (!isnan(m_Temperature))
                   out << "freeture_acq_thread_camera_sensor_temperature{description=\"Camera sensor temperature [C]\"} " << m_Temperature << endl;

               out << "freeture_acq_thread_sunrise_start{description=\"Sunrise start (sun ephemeris) in seconds starting from 00:00:00->0\"} " << m_StartSunriseTime << endl;
               out << "freeture_acq_thread_sunset_start{description=\"Sunset start (sun ephemeris) in seconds starting from 00:00:00->0\" } " << m_StartSunsetTime << endl;
              
               out << "freeture_acq_thread_sunrise_end{description=\"Sunrise end (sun ephemeris) in seconds starting from 00:00:00->0\"} " << m_StopSunriseTime << endl;
               out << "freeture_acq_thread_sunset_end{description=\"Sunset end (sun ephemeris) in seconds starting from 00:00:00->0\" } " << m_StopSunsetTime << endl;

               out.close();

               m_Writing = false;
           }
        }
        catch(exception& e)
        {
            cout << e.what() << endl;
        }
}

NodeExporterMetrics::NodeExporterMetrics(std::string station_name)
    : OutputPath("/freeture/"+ station_name + "/" + DEFAULT_METRICS_OUTPUT_FILE),
    m_StationName(station_name)
{
}
