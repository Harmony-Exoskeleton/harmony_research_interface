/** @file
 * @brief Contains definitions of functions used for handling exceptions when reading and loading parameters from external configuration files.
*/


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "file_exception_handling.h"
#include "data_types_handling.h"
#include <iostream>

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/** @brief Counts number of occurrences of a key in a file (only used for csv files)
 *
 * @param key specified by a string
 * @param file specified by a fstream
 * @return number of occurrences of a key in a file
 */
unsigned config_file_exception_handler::count_key_occurrences(const std::string key, std::ifstream &file)
{
    std::string word;
    unsigned count = 0;

    file.clear();
    file.seekg(0,std::ios::beg);

    while(file)
    {
        file>>word;

        if(word.find(key.c_str())!=std::string::npos)
        {
            count++;
        }
    }
    return count;
}


/** @brief Returns file size in Bytes
 *
 * @param file specified by a fstream
 * @return size of file
 */
std::streampos config_file_exception_handler::get_fstream_size(std::ifstream &file)
{
    std::streampos begin, end;
    begin = file.tellg();
    file.seekg (0, std::ios::end);
    end = file.tellg();

    file.seekg (0);

    return end-begin;
}

/** @brief Returns file extension
 *
 * @param filename specified by a string
 * @return extension of filename
 */
std::string config_file_exception_handler::get_extension(const std::string filename)
{
    std::string extension;
    extension = filename.substr(filename.find_last_of(".") + 1);

    return extension;
}


/** @brief Verifies if file is valid
 *
 * It verifies if file exists, if it's empty, or if it's too large.
 * @param file specified by a fstream
 * @return handle describing the status of the fstream error handler (error: <0, success: 0, warning >0)
 */
config_file_exception config_file_exception_handler::fstream_error_handling(std::ifstream &file)
{
    //File exists?
    if(!file)
    {
        file.close();
        config_file_exception exception_code = config_file_exception::ERR_FILE_NONEXISTENT;
        PLOGE << get_exception_message(exception_code);
        return exception_code;
    }

    //Empty?
    if(file.peek() == std::ifstream::traits_type::eof())
    {
        file.close();
        config_file_exception exception_code = config_file_exception::ERR_FILE_EMPTY;
        PLOGE << get_exception_message(exception_code);
        return exception_code;
    }

    //Too large?
    if(get_fstream_size(file)>MAX_FILE_SIZE)
    {
        file.close();
        config_file_exception exception_code = config_file_exception::ERR_FILE_SIZE;
        PLOGE << get_exception_message(exception_code);
        return exception_code;
    }

    return config_file_exception::SUCCESS;
}


/** @brief Returns exception message
 *
 * @param exception code
 * @return error message for a given code
 */
std::string config_file_exception_handler::get_exception_message(const config_file_exception code)
{
    std::string output;

    switch(code)
    {
        case config_file_exception::ERR_MATRIX_SIZE:
            output = "ERR_MATRIX_SIZE: Process stopped. Matrix size is too large to be loaded!(Max file size: " + std::to_string(MAX_MATRIX_SIZE) + "Bytes)";
            break;
        case config_file_exception::ERR_FILE_NOT_CREATED:
            output = "ERR_FILE_NOT_CREATED: Process stopped. Requested file could not be created!";
            break;
        case config_file_exception::ERR_FILE_SIZE:
            output = "ERR_FILE_SIZE: Process stopped. Requested file too large to be loaded! (Max file size: " + std::to_string(MAX_FILE_SIZE) + "Bytes)";
            break;
        case config_file_exception::ERR_FILE_EMPTY:
            output = "ERR_FILE_EMPTY : Process stopped. Requested file is empty!";
            break;
        case config_file_exception::ERR_FILE_NONEXISTENT:
            output = "ERR_FILE_NONEXISTENT: Process stopped. Requested file does not exist!";
            break;
        case config_file_exception::ERR_FILE_FORMAT:
            output = "ERR_FILE_FORMAT: Process stopped. File format not readable or not found! (Requested file must have one of the following extensions: .json or .csv)" ;
            break;
        case config_file_exception::ERR_PARAM_NOT_FOUND:
            output = "ERR_PARAM_NOT_FOUND: Process stopped. The parameter requested was not found!";
            break;
        case config_file_exception::ERR_TYPE_MISMATCH:
            output = "ERR_TYPE_MISMATCH: Process stopped. The value of the requested parameter does not match the expected type!";
            break;
        case config_file_exception::ERR_PARAM_NOT_LOADED:
            output = "ERR_PARAM_NOT_LOADED: Process stopped. The requested parameter could not be loaded!";
            break;
        case config_file_exception::ERR_TYPE_UNKNOWN:
            output = "ERR_TYPE_UNKNOWN: Process stopped. Parameter type not recognized!";
            break;
        case config_file_exception::ERR_FILE_PARSING:
            output = "ERR_FILE_PARSING: Process stopped. Error parsing the file!";
            break;
        case config_file_exception::ERR_JSON_MISMATCH:
            output = "ERR_JSON_MISMATCH: Process stopped. Unexpected file format! Choose a file with .json extension!";
            break;
        case config_file_exception::ERR_CSV_MISMATCH:
            output = "ERR_CSV_MISMATCH: Process stopped. Unexpected file format! Choose a file with .csv extension!";
            break;
        case config_file_exception::ERR_JSON_HIERARCHY:
            output = "ERR_JSON_HIERARCHY: Process stopped. Json object of level 1 identified!";
            break;
        case config_file_exception::WRNG_INCOMPLETE_PARAM:
            output = "WRNG_INCOMPLETE_PARAM: Parameters in the requested file have a missing or incomplete name or value fields!";
            break;
        case config_file_exception::WRNG_PARAM_PARSING:
            output = "WRNG_PARAM_PARSING: Parameters in the requested file could not be parsed correctly!";
            break;
        case config_file_exception::WRNG_NUM_PARAM:
            output = "WRNG_NUM_PARAM: The requested file contains a number of objects different from the expected!" ;
            break;
        case config_file_exception::WRNG_PARAM_DUPLICATE:
            output = "WRNG_PARAM_DUPLICATE: The requested file contains parameters with multiple occurrences! ";
            break;
        case config_file_exception::ERR_PARAM_BOUNDS:
            output = "ERR_PARAM_BOUNDS: The requested parameter is out of bounds! ";
            break;
        case config_file_exception::WRNG_FILE_NOT_OPEN:
            output = "WRNG_FILE_NOT_OPEN: Data not logged. File is not open.";
            break;
        default:
            output = "";
            break;

    }
    return output;
}


/** @brief Verifies if vector has a NaN
 *
 * @param value specified by a Eigen::Vector3d
 * @return true if at at least one element is NaN, false otherwise
 */
bool config_file_exception_handler::is_value_nan(Eigen::Vector3d value)
{
    for(unsigned i=0;i<3;i++)
    {
        if(isnan(value(i)))
            return true;
    }

    return false;
}


/** @brief Verifies if matrix has a NaN
 *
 * @param value specified by a Eigen::Matrix3d
 * @return true if at at least one element is NaN, false otherwise
 */
bool config_file_exception_handler::is_value_nan(Eigen::Matrix3d value)
{
    for(unsigned i=0;i<3;i++)
    {
        for(unsigned j=0;j<3;j++)
        {
            if(isnan(value(i,j)))
                return true;
        }
    }

    return false;
}


/** @brief Verifies if json object has a NaN
 *
 * @param value specified by a basic_json
 * @return true if at at least one element is NaN, false otherwise
 */
bool config_file_exception_handler::is_value_nan(nlohmann::json value)
{
    nlohmann::json input(value.flatten());

    for(auto it=input.begin();it!=input.end();++it)
    {
        if(isnan(double(it.value())))
            return true;
    }

    return false;
}


/** @brief Check if scalar parameter is out of bounds.
 *
 * @param parameter value specified by a double
 * @param parameter name specified by a string
 * @return handle indicating if it is out of bound
 */
template<typename T>
config_file_exception config_file_exception_handler::is_scalar_out_of_bounds(const double var, const std::string name)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    if(is_mass_type<T>::value)
    {
        if(var < bounds::MASS_LOWER || var > bounds::MASS_UPPER)
        {
            config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
            PLOGE << get_exception_message(exception) << " " << name << ": " << var << ", Bounds: [" << bounds::MASS_LOWER << "," << bounds::MASS_UPPER << "]"
                  << "\nFile: " << get_filename() << ", Object: " << get_object_name();
            error = exception;

            return error;
        }
    }
    else if(is_length_type<T>::value)
    {
        if(var < bounds::LENGTH_LOWER || var > bounds::LENGTH_UPPER)
        {
            config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
            PLOGE << get_exception_message(exception) << " " << name << ": " << var << ", Bounds: [" << bounds::LENGTH_LOWER << "," << bounds::LENGTH_UPPER << "]"
                  << "\nFile: " << get_filename() << ", Object: " << get_object_name();
            error = exception;

            return error;
        }
    }
    else if(is_angle_type<T>::value)
    {
        if(var < bounds::ANGLE_LOWER || var > bounds::ANGLE_UPPER)
        {
            config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
            PLOGE << get_exception_message(exception) << " " << name << ": " << var << ", Bounds: [" << bounds::ANGLE_LOWER << "," << bounds::ANGLE_UPPER << "]"
                  << "\nFile: " << get_filename() << ", Object: " << get_object_name();
            error = exception;

            return error;
        }
    }
    else if(is_sign_type<T>::value)
    {
        if(!(var == 1 || var == -1))
        {
            config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
            PLOGE << get_exception_message(exception) << " " << name << ": " << var << ", Select from: {" << -1 << "," << 1 << "}"
                  << "\nFile: " << get_filename() << ", Object: " << get_object_name();
            error = exception;

            return error;
        }
    }
    else if(is_selector_type<T>::value)
    {
        if(!(var == 1 || var == 0))
        {
            config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
            PLOGE << get_exception_message(exception) << " " << name << ": " << var << ", Select from: {" << 0 << "," << 1 << "}"
                  << "\nFile: " << get_filename() << ", Object: " << get_object_name();
            error = exception;

            return error;
        }
    }
    return no_error;
}
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<_int>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<_double>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<_float>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<_uint16_t>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<_int16_t>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<mass_t>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<angle_t>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<sign_t>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<selector_t>(const double,const std::string);
template config_file_exception config_file_exception_handler::is_scalar_out_of_bounds<length_t>(const double,const std::string);


/** @brief Check if vector parameter is out of bounds.
 *
 * @param parameter value specified by a Vector3d
 * @param parameter name specified by a string
 * @return handle indicating if it is out of bound
 */
template<typename T>
config_file_exception config_file_exception_handler::is_vector_out_of_bounds(const Eigen::Vector3d var, const std::string name)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    if(is_center_of_gravity_type<T>::value)
    {
        for(unsigned i=0;i<3;i++)
        {
           if(var(i) < bounds::COG_LOWER || var(i) > bounds::COG_UPPER)
           {
               config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
               PLOGE << get_exception_message(exception) << "\n" << name << ": " << var << "\nBounds: [" << bounds::COG_LOWER << "," << bounds::COG_UPPER << "]"
                     << "\nFile: " << get_filename() << ", Object: " << get_object_name();
               error = exception;

               return error;
           }
        }
    }
    return no_error;
}
template config_file_exception config_file_exception_handler::is_vector_out_of_bounds<center_of_gravity_t>(const Eigen::Vector3d,const std::string);


/** @brief Check if matrix parameter is out of bounds.
 *
 * @param parameter value specified by a MatrixXd
 * @param parameter name specified by a string
 * @return handle indicating if it is out of bound
 */
template<typename T>
config_file_exception config_file_exception_handler::is_matrix_out_of_bounds(Eigen::MatrixXd var, const std::string name)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    if(is_tensor_type<T>::value)
    {
        for(unsigned i=0;i<var.rows();i++)
        {
            for(unsigned j=0;j<var.cols();j++)
            {
                if(var(i,j) < bounds::TENSOR_LOWER || var(i,j) > bounds::TENSOR_UPPER)
                {
                    config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
                    PLOGE << get_exception_message(exception) << "\n" << name << ": " << var << "\nBounds: [" << bounds::TENSOR_LOWER << "," << bounds::TENSOR_UPPER << "]"
                          << "\nFile: " << get_filename() << ", Object: " << get_object_name();
                    error = exception;

                    return error;
                }
            }
        }
    }

    if(is_angle_matrix_type<T>::value)
    {
        for(unsigned i=0;i<var.rows();i++)
        {
            for(unsigned j=0;j<var.cols();j++)
            {
                if(var(i,j) < bounds::ANGLE_LOWER || var(i,j) > bounds::ANGLE_UPPER)
                {
                    config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
                    PLOGE << get_exception_message(exception) << "\n" << name << ": " << var << "\nBounds: [" << bounds::ANGLE_LOWER << "," << bounds::ANGLE_UPPER << "]"
                          << "\nFile: " << get_filename() << ", Object: " << get_object_name();
                    error = exception;

                    return error;
                }
            }
        }
    }


    if(is_length_matrix_type<T>::value)
    {
        for(unsigned i=0;i<var.rows();i++)
        {
            for(unsigned j=0;j<var.cols();j++)
            {
                if(var(i,j) < bounds::LENGTH_LOWER || var(i,j) > bounds::LENGTH_UPPER)
                {
                    config_file_exception exception = config_file_exception::ERR_PARAM_BOUNDS;
                    PLOGE << get_exception_message(exception) << "\n" << name << ": " << var << "\nBounds: [" << bounds::LENGTH_LOWER << "," << bounds::LENGTH_UPPER << "]"
                          << "\nFile: " << get_filename() << ", Object: " << get_object_name();
                    error = exception;

                    return error;
                }
            }
        }
    }

    return no_error;
}
template config_file_exception config_file_exception_handler::is_matrix_out_of_bounds<_Matrix>(const Eigen::MatrixXd,const std::string);
template config_file_exception config_file_exception_handler::is_matrix_out_of_bounds<angle_matrix_t>(const Eigen::MatrixXd,const std::string);
template config_file_exception config_file_exception_handler::is_matrix_out_of_bounds<length_matrix_t>(const Eigen::MatrixXd,const std::string);
template config_file_exception config_file_exception_handler::is_matrix_out_of_bounds<tensor_t>(const Eigen::MatrixXd,const std::string);

