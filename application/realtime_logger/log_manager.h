#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H

#include "csv_handling.h"

namespace logger{
//std::string create_header(std::vector<std::string> joints_labels);
std::string create_header();
std::string create_dataline(const double elapsed_time, const int mode);
}

#endif // LOG_MANAGER_H
