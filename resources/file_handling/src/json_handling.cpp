/** @file
 * @brief Contains definitions of functions used for reading and loading parameters from json configuration files.
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "json_handling.h"


/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/


/** @brief Parses json objects from a file.
 *
 * This makes objects assessible to other functions. This function calls nlohmann parser.
 * Beware that if the file has multiple occurrences of an object name, the parser only stores one of the instances.
 * nlohmann documentation does not specify which instance will be stored.
 * The same applies for multiple instances of the same parameter within a parent object.
 * @param filename including path containing the parameters
 * @param number of objects expected in the file (consisting of structs of parameters)
 * @return handle describing the status of the parsing procedure (error: <0, success: 0, warning >0)
 */
config_file_exception i_json_data_file::parse(const std::string filename, int num_objects)
{
    exception_handler.set_filename(filename);
    root_filename = filename;

    std::ifstream file(filename);
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;
    config_file_exception exception;

    exception = exception_handler.fstream_error_handling(file);

    if(static_cast<int>(exception) > 0)
    {
        no_error = exception; //This is a warning and does not stop excecution
    }
    else if(static_cast<int>(exception) < 0)  //This is an error and it does stop excecution
    {
        error = exception;
        return error;
    }

    std::string extension = exception_handler.get_extension(filename);

    if(!(extension.compare("json")==0)) //This is an error and it does stop excecution
    {
        error = config_file_exception::ERR_JSON_MISMATCH;
        return error;
    }
    else {
        exception =  read_file(file, num_objects);
        if(static_cast<int>(exception) > 0)
        {
            no_error = exception; //This is a warning and does not stop excecution
        }
        else if(static_cast<int>(exception) < 0) //This is an error and it does stop excecution
        {
            error = exception;
            return error;
        }
    }

    return no_error;
}

/** @brief Verifies and reads fstream file to a root json object. This function calls nlohmann parser.
 * Beware that if the file has multiple occurrences of an object name, the parser only stores one of the instances.
 * nlohmann documentation does not specify which instance will be stored.
 * The same applies for multiple instances of the same parameter within a parent object.
 * @param fstream storing file contents
 * @param number of objects expected in the file (consisting of structs of parameters)
 * @return handle describing the status of the file reading/parsing procedure (error: <0, success: 0, warning >0)
 */
config_file_exception i_json_data_file::read_file(std::ifstream &file, int num_objects)
{

    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;
    config_file_exception exception;

    try {
        file >> root;
    } catch (nlohmann::detail::parse_error exception) {
        file.close();
        error = config_file_exception::ERR_FILE_PARSING;
        PLOGE << exception.what() << "\n" << exception_handler.get_exception_message(error);

        return error;
    }

    exception =  file_error_handling(num_objects);

    if(static_cast<int>(exception) > 0) //This is a warning and does not stop excecution
    {
        no_error = exception;
    }
    if(static_cast<int>(exception) < 0) //This is an error and it does stop excecution
    {
        file.close();

        error = exception;
        return error;
    }

    file.close();
    return no_error;
}


void json_object::set_exception_labels(const std::string filename, const std::string object_name)
{
    exception_handler.set_object_name(object_name);
    exception_handler.set_filename(filename);
}

/** @brief Populates the parameter list (class member)
 *
 * @param json object (previously parsed from a file)
 */
void json_object::populate(nlohmann::json my_object)
{
    parameter_list = my_object;
}


/** @brief Returns value of a scalar parameter specified by a key and type
 *
 * It casts the parameter type accordingly. If casting a primitive type, use _<type>.
 * Accepted types: _int, _double, _float, _uint16_t, _int16_t, sign_t, mass_t, angle_t, selector_t, length_t.
 * It throws an exception if the error handler encounters an critical error (error <0).
 * @param parameter name specified by a string
 * @return parameter value with the specified type (Note that Sign_t value is an int,
 * Selector_t value is a bool, Mass_t and Angle_t values are double, and Param_t value
 * is the type specified by the template: int, double, float, uint16_t or int16_t)
 */
template <class T>
T json_object::get_scalar(const std::string key)
{
    double scalar;
    T result;

    //Check if parsing was successful
    try {
        scalar = get_scalar_double(key); //double
    } catch (config_file_exception exception) {
        throw exception;
    }

    //Check if out of bounds
    config_file_exception exception = exception_handler.is_scalar_out_of_bounds<T>(scalar,key);
    if(static_cast<int>(exception) < 0) // It is out of bounds: throw exception
    {
        throw exception;
    }

    result.set(scalar);

    //@Ana: Delete later
    //std::cout << key << ": " << result.value << std::endl;

    return result;
}
template _int json_object::get_scalar<_int>(const std::string);
template _double json_object::get_scalar<_double>(const std::string);
template _float json_object::get_scalar<_float>(const std::string);
template _uint16_t json_object::get_scalar<_uint16_t>(const std::string);
template _int16_t json_object::get_scalar<_int16_t>(const std::string);
template sign_t json_object::get_scalar<sign_t>(const std::string);
template mass_t json_object::get_scalar<mass_t>(const std::string);
template angle_t json_object::get_scalar<angle_t>(const std::string);
template selector_t json_object::get_scalar<selector_t>(const std::string);
template length_t json_object::get_scalar<length_t>(const std::string);


/** @brief Returns value of a matrix parameter specified by a key and type
 *
 * It casts the parameter type accordingly. Accepted types: Tensor_t, _Matrix, angle_matrix_t.
 * It throws an exception if the error handler encounters an critical error (error <0).
 * @param parameter name specified by a string
 * @param number of rows specified by an unsigned integer
 * @param number of columns specified by an unsigned integer
 * @return parameter value with the specified type (Note that Tensor_t value is an Eigen::Matrix3d)
 */
template <typename T>
T json_object::get_matrix(const std::string key, const unsigned rows, const unsigned cols)
{
    Eigen::MatrixXd matrix(rows,cols);
    T result;

    //Check if size is 3x3 if type is tensor
    if(is_tensor_type<T>::value && (rows != 3 || cols != 3))
    {
        config_file_exception exception = config_file_exception::ERR_TYPE_MISMATCH;
        std::string message = exception_handler.get_exception_message(exception) + " Type expected for inertia tensor " + key + ": 3x3 matrix. But size requested is: "  + std::to_string(rows) + "x" + std::to_string(cols) +  " matrix.";
        PLOGE << message;
        throw exception;
    }
    else if(rows*cols > MAX_MATRIX_SIZE)
    {
        config_file_exception exception = config_file_exception::ERR_MATRIX_SIZE;
        std::string message = exception_handler.get_exception_message(exception) + " Size requested is: "  + std::to_string(rows) + "x" + std::to_string(cols) +  " matrix.";
        PLOGE << message;
        throw exception;
    }

    //Check if parsing was successful
    try {
        matrix = get_matrix_X_rows_Y_cols(key,rows,cols);
    } catch (config_file_exception exception) {
        throw exception;
    }

    //Check if out of bounds
    config_file_exception exception = exception_handler.is_matrix_out_of_bounds<T>(matrix,key);
    if(static_cast<int>(exception) < 0) // It is out of bounds: throw exception
    {
        throw exception;
    }

    result.set(matrix);

    //@Ana: Delete later
    //std::cout << key << ": " << result.value << std::endl;

    return result;
}
template _Matrix json_object::get_matrix<_Matrix>(const std::string, unsigned rows, unsigned cols);
template angle_matrix_t json_object::get_matrix<angle_matrix_t>(const std::string, unsigned rows, unsigned cols);
template length_matrix_t json_object::get_matrix<length_matrix_t>(const std::string, unsigned rows, unsigned cols);
template tensor_t json_object::get_matrix<tensor_t>(const std::string,unsigned rows, unsigned cols);


/** @brief Returns value of a vector parameter specified by a key and type
 *
 * It casts the parameter type accordingly. Accepted types: CenterOfGravity_t.
 * It throws an exception if the error handler encounters an critical error (error <0).
 * @param parameter name specified by a string
 * @return parameter value with the specified type (Note that CenterOfGravity_t value is an Eigen::Vector3d)
 */

template <class T>
T json_object::get_vector(const std::string key)
{
    Eigen::Vector3d vector;
    T result;

    //Check if parsing was successful
    try {
        vector = get_vector3d(key);
    } catch (config_file_exception exception) {
        throw exception;
    }

    //Check if out of bounds
    config_file_exception exception = exception_handler.is_vector_out_of_bounds<T>(vector,key);
    if(static_cast<int>(exception) < 0) // It is out of bounds: throw exception
    {
        throw exception;
    }

    result.set(vector);

    //@Ana: Delete later
    //std::cout << key << ": " << result.value << std::endl;

    return result;
}
template center_of_gravity_t json_object::get_vector<center_of_gravity_t>(const std::string);


/** @brief Returns value of a scalar specified by a key
 *
 * It returns a double.
 * It throws an exception if the error handler encounters an critical error (error <0).
 * @param parameter name specified by a string
 * @return parameter value as a double
 */
double json_object::get_scalar_double(const std::string key)
{
    double output;

    config_file_exception exception =  error_handling(key, "scalar");

    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        throw exception;
    }

    output = parameter_list[key];

    return output;
}

/** @brief Returns value of a vector specified by a key
 *
 * It returns a Eigen::Vector3d. * 
 * It throws an exception if the error handler encounters an critical error (error <0).
 * @param parameter name specified by a string
 * @return parameter value as an Eigen::Vector3d
 */
Eigen::Vector3d json_object::get_vector3d(const std::string key)
{
    Eigen::Vector3d output = Eigen::Vector3d::Zero()*nan(""); //Initialize with NaN (facilitates check for incomplete)

    config_file_exception exception =  error_handling(key, "vector");

    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        throw exception;
    }

    for(unsigned i=0; i<parameter_list[key].flatten().size(); i++)
        output(i) = parameter_list[key][i];

    return output;
}


/** @brief Returns a matrix specified by a key and the number of rows and columns
 *
 * It returns a Eigen::MatrixXd.
 * @param parameter name specified by a string
 * @param number of rows specified by an unsigned integer
 * @param number of columns specified by an unsigned integer
 * @return parameter value as an Eigen::MatrixXd
 */
Eigen::MatrixXd json_object::get_matrix_X_rows_Y_cols(const std::string key,const unsigned X,const unsigned Y)
{
    Eigen::MatrixXd output = Eigen::MatrixXd::Zero(X,Y)*nan(""); //Initialize with NaN (facilitates check for incomplete)

    config_file_exception exception =  error_handling(key, "matrix", X*Y);

    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        throw exception;
    }

    if(X==1) //This is a row
    {
        for(unsigned j=0; j<Y; j++)
            output(j) = parameter_list[key][j];
    }
    else if(Y==1) //This is a column
    {
        for(unsigned i=0; i<X; i++)
            output(i) = parameter_list[key][i];

    }
    else
    {
        for(unsigned i=0; i<X; i++)
        {
            for(unsigned j=0; j<Y; j++)
                output(i,j) = parameter_list[key][i][j];
        }
    }

    return output;
}

/** @brief Verifies if parameter specified by a key and type is valid.
 *
 * Checks if parameter value is null and if its format matches the specified type.
 * @param parameter name specified by a string
 * @param parameter type specified by a string chosen from: {scalar, vector, matrix}
 * @return handle describing the status of the error handler (error: <0, success: 0, warning >0)
 */
config_file_exception json_object::error_handling(const std::string key, const std::string type, const unsigned elements)
{    
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    //Null?
    if(parameter_list[key].is_null())
    {
        config_file_exception exception = config_file_exception::ERR_PARAM_NOT_FOUND;
        std::string message = exception_handler.get_exception_message(exception);
        message += "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;

        PLOGE << message;
        error = exception;

        return error;
    }

    //Matches with scalar?
    if(type.compare("scalar") == 0)
    {
        if(!parameter_list[key].is_number())
        {
            config_file_exception exception = config_file_exception::ERR_TYPE_MISMATCH;
            std::string message = exception_handler.get_exception_message(exception) + " Type expected: " + type+ ". But value is: " + parameter_list[key].type_name();
            message += "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;

            PLOGE << message;
            error = exception;

            return error;
        }

        else if(exception_handler.is_value_nan(parameter_list[key].flatten()))
        {
            config_file_exception exception = config_file_exception::WRNG_INCOMPLETE_PARAM;
            no_error = exception;

            PLOGW << " Parameter is incomplete! Completing with NaN!";
            PLOGW << "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;
        }
    }
    else
    {
        //Undefined parameter type?
        if(!parameter_list[key].is_array())
        {
            config_file_exception exception = config_file_exception::ERR_TYPE_UNKNOWN;
            std::string message = exception_handler.get_exception_message(exception) + " Type expected: " + type+ ". But value is: " + parameter_list[key].type_name();
            message += "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;

            PLOGE << message;
            error = exception;

            return error;
        }

        //Matches with vector?
        if(type.compare("vector") == 0)
        {
            if(parameter_list[key].flatten().size()!=3) //size must be 3 for vector
            {
                config_file_exception exception = config_file_exception::ERR_TYPE_MISMATCH;
                std::string message = exception_handler.get_exception_message(exception) + " Type expected: 3d " + type + ". But value is: " + parameter_list[key].type_name();
                message += "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;

                PLOGE << message;
                error = exception;

                return error;
            }

            else if(exception_handler.is_value_nan(parameter_list[key].flatten())) //complete with NaN
            {
                config_file_exception exception = config_file_exception::WRNG_INCOMPLETE_PARAM;
                no_error = exception;

                PLOGW << " Parameter is incomplete! Completing with NaN!";
                PLOGW << "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;
            }
        }

        //Matches with matrix?
        else if(type.compare("matrix") == 0)
        {
            if(parameter_list[key].flatten().size()!=elements) //size must be "elements" for matrix
            {
                config_file_exception exception = config_file_exception::ERR_TYPE_MISMATCH;
                std::string message = exception_handler.get_exception_message(exception) + " Type expected: " + type + " with " + std::to_string(elements) + " elements. But value is: " + parameter_list[key].type_name() + " with " + std::to_string(parameter_list[key].flatten().size()) + " elements.";
                message += "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;

                PLOGE << message;
                error = exception;

                return error;
            }
            else if(exception_handler.is_value_nan(parameter_list[key])) //complete with NaN
            {
                config_file_exception exception = config_file_exception::WRNG_INCOMPLETE_PARAM;
                no_error = exception;

                PLOGW << " Parameter is incomplete! Completing with NaN!";
                PLOGW << "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;
            }
        }
        else //Undefined specified type?
        {
            config_file_exception exception = config_file_exception::ERR_TYPE_UNKNOWN;
            std::string message = exception_handler.get_exception_message(exception) + " Type expected: " + type+ ". But value is: " + parameter_list[key].type_name();
            message += "\nFile: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name() + ", Parameter: " + key;

            PLOGE << message;
            error = exception;

            return error;
        }
    }

    return no_error;
}

/** @brief Returns json object specified by a key.
 *
 * It throws an exception if the object handler encounters a critical error (error <0)
 * @param object name specified by a string
 * @return json object found
 */
json_object i_json_data_file::get_object(const std::string key)
{
    config_file_exception exception = object_error_handling(key);

    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        throw exception;
    }

    json_object object;
    object.set_exception_labels(exception_handler.get_filename(), key);
    object.populate(root[key]);

    return object;
}

unsigned i_json_data_file::get_number_objects()
{
    return root.size();
}

/** @brief Verifies if object specified by a key is valid.
 *
 * It verifies is object is null and if it is an object and not a parameter.
 * @param object name specified by a string
 * @return handle describing the status of the object error handler (error: <0, success: 0, warning >0)
 */
config_file_exception i_json_data_file::object_error_handling(const std::string key)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    //Null?
    if(root[key].is_null())
    {
        config_file_exception exception = config_file_exception::ERR_PARAM_NOT_FOUND;
        PLOGE << exception_handler.get_exception_message(exception) << ". File: " << exception_handler.get_filename() << ", Object: " << key;
        error = exception;

        return error;
    }

    //Not an object (level 1)?
    if(!root[key].is_object())
    {
        config_file_exception exception = config_file_exception::ERR_JSON_HIERARCHY;
        PLOGE << exception_handler.get_exception_message(exception) << ". File: " << exception_handler.get_filename() << ", Object: " << key;
        error = exception;

        return error;
    }

    return no_error;
}

/** @brief Verifies if json file is valid.
 *
 * It checks if all parameters are valid and if the number of objects matches expectations.
 * @param number of expected objects
 * @return handle describing the status of the file error handler (error: <0, success: 0, warning >0)
 */
config_file_exception i_json_data_file::file_error_handling(int num_objects_expected)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;
    int num_objects_real;

    num_objects_real = root.size();

    config_file_exception exception;
    //Check all parameters (iterate per object)
    for(auto it=root.begin();it!=root.end();++it)
    {
        exception = get_object(it.key()).check_parameters();
        if(static_cast<int>(exception) < 0)  //This is an error and should throw an exception
        {
            error = exception;
            return error;
        }
    }

    //Check number of objects
    if(num_objects_expected < 0)
    {
        exception = config_file_exception::WRNG_NUM_PARAM;
        num_objects_expected = num_objects_real;
        no_error = exception;

        //@Ana: This is going to go off every time because exercise library does not specify a number of objects. Removing the warning
        //PLOGW << "Number of objects was not specified. Parsed all " + std::to_string(num_objects_real) + " objects. File: " + exception_handler.get_filename();
    }

    if(num_objects_real!=num_objects_expected)
    {
        exception = config_file_exception::WRNG_NUM_PARAM;
        std::string message;
        message = exception_handler.get_exception_message(exception) + " Expected: " + std::to_string(num_objects_expected) + " objects. Parsed: " + std::to_string(num_objects_real) + " objects.";
        message += " File: " + exception_handler.get_filename();

        no_error = exception;
        PLOGW << message;
    }

    return no_error;
}

/** @brief Verifies if all parameters associated with a json object are valid.
 *
 * It initialize parameter values with NaN so it can easily catch incomplete instances. Check if parameter values are empty.
 * Currently does not return errors, just warnings, but it is structured to allow so in the future.
 * @return handle describing the status of the parameter handler (error: <0, success: 0, warning >0)
 */
config_file_exception json_object::check_parameters()
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error; //not used

    bool incomplete_param_exception_encountered = false;
    bool parsing_exception_encountered = false;
    std::vector<std::string> incomplete_parameters;
    std::vector<std::string> parse_error_parameters;

    //Initialize with NaN
    double vector_param_value[3] = {nan(""), nan(""), nan("")};

    unsigned count = 0;
    for(auto it = parameter_list.begin();it!=parameter_list.end();++it)
    {
        //Incomplete? (Parameter incomplete exception - see if matrix or vector3d is incomplete)
        //Complete with NaN
        if(it.value().is_structured())
        {
            unsigned cols = 0;
            unsigned rows = it.value().size();
            if(it.value()[0].is_array())
            {
                cols = it.value()[0].size();
                bool matrix_incomplete = false;
                for(unsigned i=1;i<rows;i++)
                {
                    if(cols!=it.value()[i].size())
                    {
                        matrix_incomplete = true;
                        if(it.value()[i].size()>cols)
                            cols = it.value()[i].size();
                    }
                }

                if(matrix_incomplete)
                {
                    nlohmann::json matrix_param_value = nlohmann::json::array();

                    for (unsigned i=0; i<rows;i++)
                    {
                        nlohmann::json temp = nlohmann::json::array();
                        for (unsigned j=0; j<it.value()[i].size();j++)
                        {
                            temp += it.value()[i][j];
                        }
                        for (unsigned j=it.value()[i].size(); j<cols;j++)
                        {
                            temp += nan("");
                        }
                        matrix_param_value += temp;
                    }
                    nlohmann::json object = matrix_param_value;
                    it->swap(object);
                    std::string message_it = "Parameter " + it.key() + ": " + std::to_string(it.value().flatten().size())  + " fields!";
                    incomplete_parameters.push_back(message_it);
                    incomplete_param_exception_encountered = true;
                }
            }
            else if(it.value().flatten().size()>1 && it.value().flatten().size()<3)
            {
                for(unsigned i = 0; i< it.value().size(); i++)
                {
                    vector_param_value[i] = it.value()[i];
                }

                nlohmann::json object = vector_param_value;
                it->swap(object);
                std::string message_it = "Parameter " + it.key() + ": " + std::to_string(it.value().size())  + " fields!";
                incomplete_parameters.push_back(message_it);
                incomplete_param_exception_encountered = true;
            }
        }

        //Value or key empty? (Parameter parsing exception)
        if((it->empty()) || (it.key().empty()))
        {
            std::string message_it = "Parameter name: " + it.key() + "";
            parse_error_parameters.push_back(message_it);
            parsing_exception_encountered = true;
        }

        count++;
    }

    if(parsing_exception_encountered)
    {
        config_file_exception exception = config_file_exception::WRNG_PARAM_PARSING;
        std::string message;
        message = exception_handler.get_exception_message(exception) + "\n";
        for(unsigned i=0;i<parse_error_parameters.size();i++)
            message += parse_error_parameters[i] + "\n";

        message += "File: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name();

        PLOGW << message;
        no_error = exception;
    }

    if(incomplete_param_exception_encountered)
    {
        config_file_exception exception = config_file_exception::WRNG_INCOMPLETE_PARAM;
        std::string message;
        message = exception_handler.get_exception_message(exception) + " Completing with NaN!\n";
        for(unsigned i=0;i<incomplete_parameters.size();i++)
            message += incomplete_parameters[i] + "\n";

        message += "File: " + exception_handler.get_filename() + ", Object: " + exception_handler.get_object_name();

        PLOGW << message;
        no_error = exception;
    }

    return no_error;
}

config_file_exception o_json_data_file::save_file(std::string filename)
{
    std::ofstream file;
    file.open(filename);

    if(file.is_open())
    {
        std::cout <<"Successfully created json file " << filename << std::endl;
    }
    else
    {
        return config_file_exception::ERR_FILE_NOT_CREATED;
    }
    std::string file_contents = reshape_contents(get_string_from_json(root));

    std::cout << file_contents << std::endl;

    file << file_contents << std::endl;

    file.close();

    return config_file_exception::SUCCESS;
}

void o_json_data_file::set_content_double(const std::string object_name, const std::string key, const double value)
{
    root[object_name][key] = value;
}


void o_json_data_file::set_content_matrix(const std::string object_name, const std::string key, const Eigen::MatrixXd value)
{
    int n_rows = value.rows();
    int n_cols = value.cols();

    if(n_rows==1 || n_cols==1)
    {
        int n_elements = n_rows*n_cols;
        std::vector<double> dummy(n_elements,0);
        root[object_name][key] = dummy;

        for(int i = 0;i<n_elements;i++)
        {
            root[object_name][key][i] = value(i);
        }
    }
    else
    {
        std::vector<std::vector<double>> dummy(n_rows, std::vector<double>(n_cols,0));
        root[object_name][key] = dummy;

        for(int i = 0;i<n_rows;i++)
        {
            for(int j=0;j<n_cols;j++)
            {

                root[object_name][key][i][j] = value(i,j);
            }
        }
    }
}

void o_json_data_file::set_content_vector(const std::string object_name, const std::string key, const Eigen::Vector3d value)
{
    int n_elements = value.size();
    std::vector<double> dummy(n_elements, 0);
    root[object_name][key] = dummy;

    for(int j=0;j<n_elements;j++)
    {
        root[object_name][key][j] = value(j);
    }
}


std::string o_json_data_file::reshape_contents(const std::string file_contents)
{

    int n_tabs = 0;
    std::string tabs = "";
    bool stop_cri = false;
    unsigned ind = 0;

    std::string reshaped_file_contents = file_contents;

    while(!stop_cri)
    {
        if(reshaped_file_contents.at(ind)=='{')
        {
            if(ind>0)
            {
                reshaped_file_contents.insert(ind,"\n"+tabs);
                ind = ind+n_tabs+1;
            }

            n_tabs++;
            tabs.append("\t");

            reshaped_file_contents.insert(ind+1,"\n"+tabs);
            ind = ind+n_tabs+1;
        }
        else if(reshaped_file_contents.at(ind)==',')
        {
            if(reshaped_file_contents.at(ind+1)=='\"')
            {
                reshaped_file_contents.insert(ind+1,"\n"+tabs);
                ind = ind+n_tabs+1;
            }
            else ind++;
        }
        else if(reshaped_file_contents.at(ind)=='}')
        {

            n_tabs--;
            tabs.erase(tabs.end()-1,tabs.end());

            reshaped_file_contents.insert(ind,"\n"+tabs);
            ind = ind+n_tabs+1;

            reshaped_file_contents.insert(ind+1,"\n"+tabs);
            ind = ind+n_tabs+1;
        }
        else ind++;

        if(ind >= reshaped_file_contents.size())
            stop_cri = true;
    }

    return reshaped_file_contents;
}
