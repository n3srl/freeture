//Write node exporter metrics file for freeture metrics.
#include <chrono>
#include <fstream>
#include <string>
#include <iostream>

#include "NodeExporterMetrics.h"
#include "Base64.h"

using namespace std;

NodeExporterMetrics* NodeExporterMetrics::m_Instance = nullptr;

NodeExporterMetrics& NodeExporterMetrics::GetInstance()
{
    if (m_Instance == nullptr)
        m_Instance = new NodeExporterMetrics();

    return *m_Instance;
}

NodeExporterMetrics::~NodeExporterMetrics()
{
    if (m_Instance != nullptr)
        delete m_Instance;
}

void NodeExporterMetrics::UpdateMetrics(int DetNbDetection, double DetTime)
{
    m_DetNbDetection = DetNbDetection;
    m_DetTime = DetTime;
}

void NodeExporterMetrics::UpdateMetrics(double AcqFPS,double AcqTime,vector<int>* NextSunrise=nullptr,vector<int>* NextSunset =nullptr)
{
            m_AcqFPS = AcqFPS;
            m_AcqTime = AcqTime;
            m_NextSunrise = NextSunrise;
            m_NextSunset = NextSunset;
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
               out <<"freeture_acq_thread_time{description=\"Camera acquisition time [ms]\"} "<< m_AcqTime << " "<<timestamp.count() << endl;
               out <<"freeture_acq_thread_fps{description=\"Camera acquisition thread frames per seconds [Hz]\"} "<< m_AcqFPS << " "<< timestamp.count() << endl;

               if (m_NextSunrise!=nullptr)
                    if (m_NextSunrise->size()>2)
                        out << "freeture_acq_thread_next_sunrise{description=\"Next sunrise\"} " << m_NextSunrise->at(0) << "h" << m_NextSunrise->at(1) << "m" << m_NextSunrise->at(2) << "s" << " "<< timestamp.count() << endl;

               if (m_NextSunset!=nullptr)
                    if (m_NextSunset->size()>2)
                        out << "freeture_acq_thread_next_sunset{description=\"Next sunset\"} "   << m_NextSunset->at(0)  << "h" << m_NextSunset->at(1)  << "m" << m_NextSunset->at(2)  << "s" << " "<< timestamp.count() << endl;

               out <<"freeture_det_thread_nb_detection{description=\"Camera detection thread current session detection count\"} "<< m_DetNbDetection << " "<< timestamp.count() << endl;
               out <<"freeture_det_thread_det_time{description=\"Camera detection thread current detection time [ms]\"} "<< m_DetTime << " "<< timestamp.count() << endl;

               if (m_cDate != "")
                    out <<"freeture_stack_thread_c_date{description=\"Camera stack thread cDate\"} "<< m_cDate<< " "<< timestamp.count() << endl;

               if (m_CompleteDataPath != "")
                    out <<"freeture_stack_thread_complete_data_path{description=\"Camera stack thread last stack local path\"} "<< m_CompleteDataPath << " "<< timestamp.count() << endl;


               out.close();

               m_Writing = false;
           }
        }
        catch(exception& e)
        {
            cout << e.what() << endl;
        }
}
