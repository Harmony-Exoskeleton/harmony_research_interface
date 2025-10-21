/** @file
 * @brief Contains declarations required to read and load parameters from csv configuration files.
*/

#ifndef CSV_HANDLING_H
#define CSV_HANDLING_H


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "file_exception_handling.h"


/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

class csv_object
{
public:
    csv_object(){}

    std::vector<std::string> get_key_list();
    std::vector<double> get_scalar_list();
    std::vector<Eigen::Vector3d> get_vector_list();
    std::vector<Eigen::Matrix3d> get_matrix_list();

    std::map<std::string, double> data_scalar; //for scalars
    std::map<std::string, Eigen::Vector3d> data_vector; //for scalars
    std::map<std::string, Eigen::Matrix3d> data_matrix; //for scalars


};

class csv_data_file
{

public:
    csv_data_file(){};

    config_file_exception parse(const std::string,const unsigned);
    double get_scalar(const std::string);
    Eigen::Vector3d get_vector(const std::string);
    Eigen::Matrix3d get_matrix(const std::string);

private:
    csv_object root;
    config_file_exception_handler exception_handler;

    config_file_exception read_file(std::ifstream&,const unsigned);
    config_file_exception object_error_handling(const std::string, const std::string);
    config_file_exception file_error_handling(std::ifstream&,const unsigned);
    config_file_exception check_for_multiple_occurrences(std::ifstream&);
    config_file_exception check_parameters(std::ifstream&,const unsigned);

};

class csv_log_file
{

public:
    csv_log_file(){rows_in_file=0;}
    ~csv_log_file(){file.close();}

    config_file_exception log_data(const std::string);
    config_file_exception create_file(const std::string);
    int get_rows_in_file(){return rows_in_file;}
    void close_file(){file.close();}


private:
    std::ofstream file;
    config_file_exception_handler exception_handler;
    int rows_in_file;

};

#endif // CSV_HANDLING_H
