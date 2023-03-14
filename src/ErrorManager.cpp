#include "CameraGigeAravis.h"

class ErrorManager
{
    public:
        static void CheckAravisError(GError** error)
        {
            if (error!=nullptr)
            {
                //todo: log error

                if (*error != nullptr)
                {
                    delete *error;
                    *error = nullptr;
                }
            }
        }

    protected:

    private:
};
