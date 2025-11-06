/** @file
 * @brief Contains declarations required to handle exceptions when reading and loading parameters from external configuration files.
*/

#ifndef FILE_exception_HANDLING_H
#define FILE_exception_HANDLING_H

/*****************************************************************************************
 * INCLUDES
****************************************************************************************/
#include <nlohmann/json.hpp>
#include <Eigen/Core>
#include <fstream>
#include <math.h>
#include <string>
#include <plog/Log.h>

/*****************************************************************************************
 * DEFINITIONS
 ****************************************************************************************/
#define MAX_FILE_SIZE 10485760 //10MB
#define MAX_ROWS_OUTPUT_FILE 1048576
#define MAX_MATRIX_SIZE 10000

//EXCEPTION TYPES
//Negative: stop execution, //Positive: throw warning, //Zero: success
enum class config_file_exception : int
{
    ERR_MATRIX_SIZE = -16,
    ERR_FILE_NOT_CREATED = -15,
    ERR_SETTING_PARAM = -14,
    ERR_PARAM_BOUNDS = -13,
    ERR_JSON_HIERARCHY = -12,
    ERR_CSV_MISMATCH = -11,
    ERR_JSON_MISMATCH = -10,
    ERR_FILE_SIZE = -9,
    ERR_FILE_EMPTY = -8,
    ERR_FILE_NONEXISTENT = -7,
    ERR_FILE_FORMAT = -6,
    ERR_PARAM_NOT_FOUND = -5,
    ERR_TYPE_MISMATCH = -4,
    ERR_PARAM_NOT_LOADED = -3,
    ERR_TYPE_UNKNOWN = -2,
    ERR_FILE_PARSING = -1,
    SUCCESS = 0,
    WRNG_INCOMPLETE_PARAM = 1,
    WRNG_PARAM_PARSING = 2,
    WRNG_NUM_PARAM = 3,
    WRNG_PARAM_DUPLICATE = 4,
    WRNG_FILE_NOT_OPEN = 5,
};

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

class config_file_exception_handler
{

public:
        config_file_exception_handler() : local_filename(""), local_object_name(""){}
        unsigned count_key_occurrences(const std::string, std::ifstream&);
        std::string get_extension(const std::string);
        config_file_exception fstream_error_handling(std::ifstream&);
        std::string get_exception_message(const config_file_exception);
        bool is_value_nan(const Eigen::Vector3d);
        bool is_value_nan(const Eigen::Matrix3d);
        bool is_value_nan(const nlohmann::basic_json<>);
        template <typename T>
        config_file_exception is_scalar_out_of_bounds(const double,const std::string);
        template <typename T>
        config_file_exception is_vector_out_of_bounds(const Eigen::Vector3d,const std::string);
        template <typename T>
        config_file_exception is_matrix_out_of_bounds(const Eigen::MatrixXd,const std::string);
        void set_filename(std::string filename){local_filename = filename;}
        void set_object_name(std::string object_name){local_object_name = object_name;}
        std::string get_filename() {return local_filename;}
        std::string get_object_name() {return local_object_name;}

private:
        std::streampos get_fstream_size(std::ifstream&);
        std::string local_filename;
        std::string local_object_name;
};

#endif // FILE_exception_HANDLING_H
