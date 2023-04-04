
#include "CameraGigeAravis.h"

class ErrorManager
{
    public:
        static void ArenaSDKError(std::string message)
        {
            std::cout  << "> ArenaSDKError - "<< message << std::endl;
        }

        static void Exception(std::exception& e)
        {
            std::cout  << "> Freeture - "<< e.what() << std::endl;
        }

        static void CheckAravisError(GError** gError)
        {

            if (gError!=nullptr)
            {
                GError* gError_ptr = *gError;

                //todo: log error

                if ( gError_ptr != nullptr)
                {
                    GError& _error = *gError_ptr;
                    std::cout  << "> GError - Domain: " << _error.domain << ", code: "<<_error.code << ", message: "<<_error.message  << std::endl;
                    //BOOST_LOG_SEV(m_Logger, error) << "> GError - Domain: " << _error.domain << ", code: "<<_error.code << ", message: "<<_error.message;
                    delete gError_ptr;
                    gError_ptr= nullptr;
                }
            }
        }

};
