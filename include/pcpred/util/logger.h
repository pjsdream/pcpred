#ifndef LOGGER_H
#define LOGGER_H


#include <stdarg.h>
#include <stdio.h>


namespace pcpred
{

class Logger
{
public:

    Logger() : verbose_(false) {}

    void setVerbose(bool flag = true) { verbose_ = flag; }

protected:

    void LOG(const char* format, ...)
    {
        if (verbose_)
        {
            va_list a_list;
            va_start(a_list, format);
            vprintf(format, a_list);
            va_end(a_list);
            fflush(stdout);
        }
    }

    bool verbose_;
};

}


#endif // LOGGER_H
