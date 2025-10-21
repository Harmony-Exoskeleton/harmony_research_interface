/** @file
 * @brief Contains definitions of functions used for reading and loading parameters from csv configuration files.
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "csv_handling.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/** @brief Returns a list of all keys in a csv file.
 *
 * @return vector of strings with all key names
 */
std::vector<std::string> csv_object::get_key_list()
{
    std::vector<std::string> keys;

    for(std::map<std::string, double>::const_iterator it = data_scalar.begin();it!=data_scalar.end();++it)
    {
        if(std::find(keys.begin(), keys.end(),it->first)==keys.end())
            keys.push_back(it->first);
    }
    for(std::map<std::string, Eigen::Vector3d>::const_iterator it = data_vector.begin();it!=data_vector.end();++it)
    {
        if(std::find(keys.begin(), keys.end(),it->first)==keys.end())
            keys.push_back(it->first);
    }
    for(std::map<std::string, Eigen::Matrix3d>::const_iterator it = data_matrix.begin();it!=data_matrix.end();++it)
    {
        if(std::find(keys.begin(), keys.end(),it->first)==keys.end())
            keys.push_back(it->first);
    }

    return keys;
}


/** @brief Returns a list of all scalars in a csv file.
 *
 * @return vector of doubles with all scalar values
 */
std::vector<double> csv_object::get_scalar_list()
{
    std::vector<double> scalars;

    for(std::map<std::string, double>::const_iterator it = data_scalar.begin();it!=data_scalar.end();++it)
        scalars.push_back(it->second);

    return scalars;
}


/** @brief Returns a list of all vectors in a csv file.
 *
 * @return vector of vectors with all vector values
 */
std::vector<Eigen::Vector3d> csv_object::get_vector_list()
{
    std::vector<Eigen::Vector3d> vectors;

    for(std::map<std::string, Eigen::Vector3d>::const_iterator it = data_vector.begin();it!=data_vector.end();++it)
        vectors.push_back(it->second);

    return vectors;
}


/** @brief Returns a list of all matrices in a csv file.
 *
 * @return vector of matrices with all matrix values
 */
std::vector<Eigen::Matrix3d> csv_object::get_matrix_list()
{
    std::vector<Eigen::Matrix3d> matrices;

    for(std::map<std::string, Eigen::Matrix3d>::const_iterator it = data_matrix.begin();it!=data_matrix.end();++it)
        matrices.push_back(it->second);

    return matrices;
}

/** @brief Parses csv parameters from a file.
 *
 * This makes objects assessible to other functions.
 * @param filename including path containing the parameters
 * @param number of parameters expected in the file (one per line)
 * @return handle describing the status of the parsing procedure (error: <0, success: 0, warning >0)
 */
config_file_exception csv_data_file::parse(const std::string filename, const unsigned num_parameters)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    std::ifstream file(filename);

    config_file_exception exception = exception_handler.fstream_error_handling(file);

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

    if(!(extension.compare("csv")==0))
    {
        exception = config_file_exception::ERR_CSV_MISMATCH;

        error = exception;
        return error;
    }
    else {
        config_file_exception exception =  read_file(file, num_parameters);

        if(static_cast<int>(exception) > 0)
        {
            no_error = exception; //This is a warning and does not stop excecution
        }
        else if(static_cast<int>(exception) < 0)  //This is an error and it does stop excecution
        {
            error = exception;
            return error;
        }
    }

    return no_error;
}


/** @brief Verifies and reads fstream file to a root csv object.
 *
 * @param fstream storing file contents
 * @param number of parameters expected in the file (one per line)
 * @return handle describing the status of the file reading and parsing procedure (error: <0, success: 0, warning >0)
 */
config_file_exception csv_data_file::read_file(std::ifstream &file, const unsigned num_parameters)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    std::vector<std::string> row;
    std::string line, word, temp;
    std::string param_name;
    Eigen::Vector3d vector_param_value;
    Eigen::MatrixXd matrix_param_value(1,9);
    double scalar_param_value;

    //Initialize with NaN
    vector_param_value << nan(""), nan(""), nan("");
    matrix_param_value << nan(""), nan(""), nan(""), nan(""), nan(""), nan(""), nan(""), nan(""), nan("");

    file.clear();
    file.seekg(0,std::ios::beg);

    while (file) {

        row.clear();

        getline(file, line);

        std::stringstream str(line);

        while (std::getline(str, word, ','))
        {
            row.push_back(word);
        }

        param_name = row[0];
        if(!param_name.empty())
        {
            //Read scalar
            if(row.size()==2)
            {
                try {
                    scalar_param_value = std::stod(row[1]);
                } catch (std::exception e) {
                    //Complete with Nan
                    PLOGW << e.what();
                    scalar_param_value = nan("");

                    config_file_exception exception = config_file_exception::WRNG_PARAM_PARSING;
                    no_error = exception;
                }

                root.data_scalar[param_name] = scalar_param_value;
            }
            //Read vector
            else if(row.size() > 2 && row.size() <=4)
            {
                for(unsigned i=0;i<row.size()-1;i++)
                {
                    try {
                        vector_param_value(i) = std::stod(row[i+1]);
                    } catch (std::exception e) {                        
                        //Complete with Nan
                        PLOGW << e.what();
                        vector_param_value(i) = nan("");

                        config_file_exception exception = config_file_exception::WRNG_PARAM_PARSING;
                        no_error = exception;
                    }

                }

                root.data_vector[param_name] = vector_param_value;
            }
            //Read matrix
            else if(row.size() > 4 && row.size() <=10)
            {
                for(unsigned i=0;i<(row.size()-1);i++)
                {
                    try {
                        matrix_param_value(i) = std::stod(row[i+1]);
                    } catch (std::exception e) {
                        //Complete with Nan
                        PLOGW << e.what();
                        matrix_param_value(i) = nan("");

                        config_file_exception exception = config_file_exception::WRNG_PARAM_PARSING;
                        no_error = exception;
                    }

                }
                matrix_param_value.resize(3,3);

                root.data_matrix[param_name] = matrix_param_value;
            }
        }

    }

    config_file_exception exception =  file_error_handling(file,num_parameters);

    if(static_cast<int>(exception) < 0)
    {
        file.close();
        error = exception;
        return error;
    }

    file.close();
    return no_error;


}

/** @brief Returns value of a scalar specified by a key
 *
 * It returns a double. Note that different from json, you cannot typecast parameters in csv files.
 * It throws an exception if the error handler encounters an critical error (error <0).
 * @param parameter name specified by a string
 * @return parameter value as a double
 */
double csv_data_file::get_scalar(const std::string key)
{
    double output;

    config_file_exception exception =  object_error_handling(key, "scalar");

    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        throw exception;
    }

    output = root.data_scalar[key];

    return output;
}

/** @brief Returns value of a vector specified by a key
 *
 * It returns a Eigen::Vector3d. Note that different from json, you cannot typecast parameters in csv files.
 * It throws an exception if the error handler encounters an critical error (error <0).
 * @param parameter name specified by a string
 * @return parameter value as an Eigen::Vector3d
 */
Eigen::Vector3d csv_data_file::get_vector(const std::string key)
{
    Eigen::Vector3d output;

    config_file_exception exception =  object_error_handling(key, "vector");

    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        throw exception;
    }

    output = root.data_vector[key];

    return output;
}

/** @brief Returns value of a matrix specified by a key
 *
 * It returns a Eigen::Matrix3d. Note that different from json, you cannot typecast parameters in csv files.
 * It throws an exception if the error handler encounters an critical error (error <0).
 * @param parameter name specified by a string
 * @return parameter value as an Eigen::Matrix3d
 */
Eigen::Matrix3d csv_data_file::get_matrix(const std::string key)
{
    Eigen::Matrix3d output;

    config_file_exception exception =  object_error_handling(key, "matrix");

    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        throw exception;
    }

    output = root.data_matrix[key];

    return output;
}

/** @brief Verifies if parameter specified by a key is valid.
 *
 * It verifies is object is null and if it its value's format matches the specified type
 * @param parameter name specified by a string
 * @return handle describing the status of the object error handler (error: <0, success: 0, warning >0)
 */
config_file_exception csv_data_file::object_error_handling(const std::string key, const std::string type)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    //Null?
    if((root.data_scalar.count(key)==0) & (root.data_vector.count(key)==0) & (root.data_matrix.count(key)==0))
    {
        config_file_exception exception = config_file_exception::ERR_PARAM_NOT_FOUND;
        std::string message = exception_handler.get_exception_message(exception) + " Parameter name: " + key + ".";
        PLOGE << message;
        error = exception;

        return error;
    }

    //Matches with scalar?
    if(type.compare("scalar") == 0)
    {
        if((root.data_scalar.count(key)==0))
        {
            config_file_exception exception = config_file_exception::ERR_TYPE_MISMATCH;
            std::string message = exception_handler.get_exception_message(exception) + " Type expected for parameter " + key + ": " + type + ".";
            PLOGE << message;
            error = exception;

            return error;
        }
        //Complete with NaN
        else if(exception_handler.is_value_nan(root.data_scalar[key]))
        {
            config_file_exception exception = config_file_exception::WRNG_INCOMPLETE_PARAM;
            no_error = exception;

            PLOGW << " Parameter " + key + " is incomplete! Completing with NaN!";
        }
    }
    //Matches with vector?
    else if(type.compare("vector") == 0)
    {
        if(root.data_vector.count(key)==0)
        {
            config_file_exception exception = config_file_exception::ERR_TYPE_MISMATCH;
            std::string message = exception_handler.get_exception_message(exception) + " Type expected for parameter " + key + ": " + type + ".";
            PLOGE << message;
            error = exception;

            return error;
        }
        //Complete with NaN
        else if(exception_handler.is_value_nan(root.data_vector[key]))
        {
            config_file_exception exception = config_file_exception::WRNG_INCOMPLETE_PARAM;
            no_error = exception;

            PLOGW << " Parameter " + key + " is incomplete! Completing with NaN!";
        }
    }
    //Matches with matrix?
    else if(type.compare("matrix") == 0)
    {
        if(root.data_matrix.count(key)==0)
        {
            config_file_exception exception = config_file_exception::ERR_TYPE_MISMATCH;
            std::string message = exception_handler.get_exception_message(exception) + " Type expected for parameter " + key + ": " + type + ".";
            PLOGE << message;
            error = exception;

            return error;
        }
        //Complete with NaN
        else if(exception_handler.is_value_nan(root.data_matrix[key]))
        {
            config_file_exception exception = config_file_exception::WRNG_INCOMPLETE_PARAM;
            no_error = exception;

            PLOGW << " Parameter " + key + " is incomplete! Completing with NaN!";
        }
    }
    else //Undefined specified type?
    {
        config_file_exception exception = config_file_exception::ERR_TYPE_UNKNOWN;
        std::string message = exception_handler.get_exception_message(exception) + " Type expected for parameter " + key + ": " + type + ".";
        PLOGE << message;
        error = exception;

        return error;
    }

    return no_error;
}


/** @brief Verifies if csv file is valid.
 *
 * It checks if all parameters are valid, if they have multiple occurrences, and if the number of parameters matches expectations.
 * @param file specified by fstream
 * @param number of expected parameters
 * @return handle describing the status of the file error handler (error: <0, success: 0, warning >0)
 */
config_file_exception csv_data_file::file_error_handling(std::ifstream& file, const unsigned num_parameters)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    config_file_exception exception = check_for_multiple_occurrences(file);
    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        error = exception;
        return error;
    }

    exception = check_parameters(file,num_parameters);
    if(static_cast<int>(exception) < 0) //This is an error and should throw an exception
    {
        error = exception;
        return error;
    }

    return no_error;
}

/** @brief Verifies if there are multiple occurrences of keys in the file.
 *
 * It checks if any parameter have multiple occurrences.
 * @param file specified by fstream
 * @return handle describing the status of the function (error: <0, success: 0, warning >0)
 */
config_file_exception csv_data_file::check_for_multiple_occurrences(std::ifstream& file)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    std::vector<std::string> keys = root.get_key_list();

    unsigned count[keys.size()];
    bool error_encountered = false;
    std::vector<std::string> repeated_parameters;

    //Check key by key
    for(unsigned i=0; i< keys.size(); i++)
    {
        count[i] = exception_handler.count_key_occurrences(keys[i], file);

        if(count[i] > 1)
        {
            std::string message_it = " Parameter " + keys[i] + ": " + std::to_string(count[i])  + " instances!";
            repeated_parameters.push_back(message_it);
            error_encountered = true;
        }
    }

    if(error_encountered)
    {
        config_file_exception exception = config_file_exception::WRNG_PARAM_DUPLICATE;
        std::string message;
        message = exception_handler.get_exception_message(exception) + "\n";
        for(unsigned i=0;i<repeated_parameters.size();i++)
            message += repeated_parameters[i] + "\n";

        PLOGW << message;
        no_error = exception;
    }

    return no_error;
}

/** @brief Verifies if all parameters are valid.
 *
 * It verifies incomplete instances md check if parameter values are empty.
 * @param file specified by fstream
 * @param number of expected parameters
 * @return handle describing the status of the function (error: <0, success: 0, warning >0)
 */
config_file_exception csv_data_file::check_parameters(std::ifstream& file, const unsigned num_parameters)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    std::vector<std::string> row;
    std::string line, word, temp;

    bool incomplete_param_exception_encountered = false;
    bool parsing_exception_encountered = false;
    std::vector<std::string> incomplete_parameters;
    std::vector<std::string> parse_error_parameters;


    file.clear();
    file.seekg(0,std::ios::beg);

    unsigned count = 0;

    while (file) {

        row.clear();

        getline(file, line);

        std::stringstream str(line);

        while (std::getline(str, word, ','))
        {
            row.push_back(word);
        }

        //Value or key empty? (Parameter parsing exception)
        if(row.size()==1 || (row.size()==2 && ((row[1].compare(" ")==0 && row[1].size()==1) || (row[1].compare("")==0 && row[1].size()==1))))
        {
            std::string message_it = "Parameter name: " + row[0] + "";
            parse_error_parameters.push_back(message_it);
            parsing_exception_encountered = true;
        }

        //Incomplete? (Parameter incomplete exception)
        if(row.size() > 1)
        {
            count++;
            if(!(row.size()==2 || row.size()==4 || row.size()==10))
            {
                std::string message_it = "Parameter " + row[0] + ": " + std::to_string(row.size()-1)  + " fields!";
                incomplete_parameters.push_back(message_it);
                incomplete_param_exception_encountered = true;
            }
        }
    }

    if(parsing_exception_encountered)
    {
        config_file_exception exception = config_file_exception::WRNG_PARAM_PARSING;
        std::string message;
        message = exception_handler.get_exception_message(exception) + "\n";
        for(unsigned i=0;i<parse_error_parameters.size();i++)
            message += parse_error_parameters[i] + "\n";

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

        PLOGW << message;
        no_error = exception;
    }

    if(num_parameters != count)
    {
        config_file_exception exception = config_file_exception::WRNG_NUM_PARAM;
        std::string message;
        message = exception_handler.get_exception_message(exception) + " Expected: " + std::to_string(num_parameters) + " parameters. Parsed: " + std::to_string(count) + " parameters.";

        PLOGW << message;
        no_error = exception;
    }

    return no_error;
}

/** @brief This method initializes the file to save data to file
 *
 */
config_file_exception csv_log_file::create_file(const std::string header)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception warning;
    rows_in_file = 0;

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char filename [80];

    strftime (filename,80,"log/harmony_data_%Y-%m-%d-%H-%M-%S.csv",now); //Saves on root (where build folder is)

    if(file.is_open()) file.close();

    file.open (filename);
    if(file.is_open())
    {
        std::cout <<"Successfully created output logging file " << filename << std::endl;
    }
    else{
        config_file_exception exception = config_file_exception::WRNG_FILE_NOT_OPEN;
        PLOGW << exception_handler.get_exception_message(exception);

        warning = exception;
        return warning;
    }

    file << header << std::endl;

    rows_in_file++;
    return no_error;
}

/** @brief This method write data into the log file
 *
 * bool write2file in harmony_app must be true for this to be executed
 *
 */
config_file_exception csv_log_file::log_data(std::string data_line)
{
    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception warning;

    if(!file.is_open())
    {
        config_file_exception exception = config_file_exception::WRNG_FILE_NOT_OPEN;
        PLOGW << exception_handler.get_exception_message(exception);

        warning = exception;
        return warning;
    }
    file << data_line << std::endl;

    rows_in_file++;

    return no_error;
}

