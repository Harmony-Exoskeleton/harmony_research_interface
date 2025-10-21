/** @file
 * @brief Contains declarations required to read and load parameters from json configuration files.
*/

#ifndef JSON_HANDLING_H
#define JSON_HANDLING_H


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "file_exception_handling.h"

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

class json_object
{
public:
    json_object(){}

    void populate(nlohmann::json);
    config_file_exception check_parameters();

    //Casting specific types
    template <typename T>
    T get_scalar(const std::string);
    template <typename T>
    T get_matrix(const std::string,unsigned rows, unsigned cols);
    template <typename T>
    T get_vector(const std::string);

    void set_exception_labels(const std::string, const std::string);

private:
    nlohmann::json parameter_list;
    config_file_exception_handler exception_handler;

    double get_scalar_double(const std::string);
    Eigen::Vector3d get_vector3d(const std::string);
    Eigen::MatrixXd get_matrix_X_rows_Y_cols(const std::string, const unsigned, const unsigned);
    config_file_exception error_handling(const std::string,const std::string, const unsigned elements = 0);
};

//Input json
class i_json_data_file
{

public:
    i_json_data_file(){}

    config_file_exception parse(std::string, int n_objects = -1);
    json_object get_object(const std::string);
    unsigned get_number_objects();
    std::string get_filename(){return root_filename;}

private:
    nlohmann::json root;
    config_file_exception_handler exception_handler;
    std::string root_filename;

    config_file_exception read_file(std::ifstream&, const int);
    config_file_exception object_error_handling(const std::string);
    config_file_exception file_error_handling(const int);

};

//Output json
class o_json_data_file
{

public:
    o_json_data_file(){}

    config_file_exception save_file(std::string);
    void set_content_double(const std::string object_name, const std::string key, const double value);
    void set_content_matrix(const std::string object_name, const std::string key, const Eigen::MatrixXd value);
    void set_content_vector(const std::string object_name, const std::string key, const Eigen::Vector3d value);

private:
    nlohmann::json root;
    config_file_exception_handler exception_handler;

    std::string get_string_from_json(nlohmann::json content){return content.dump();}
    std::string reshape_contents(const std::string);

};



#endif // JSON_HANDLING_H
