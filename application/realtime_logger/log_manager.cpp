#include "log_manager.h"

namespace logger{

std::string create_header()
{
    std::stringstream header;
    header.str("");
    header << "time_ms" << ",";
    header << "mode";

    return header.str();

}

std::string create_dataline(const double elapsed_time_ms, const int mode)
{
    std::stringstream header;
    header.str("");

    header << elapsed_time_ms << ",";
    header << mode;
    return header.str();
}

}


