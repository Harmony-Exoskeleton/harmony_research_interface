#ifndef DATA_TYPES_HANDLING_H
#define DATA_TYPES_HANDLING_H

/** @file
 * @brief Contains declarations required to handle exceptions when reading and loading parameters from external configuration files.
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <Eigen/Dense>
#include <string>
#include <plog/Log.h>

/*****************************************************************************************
 * DEFINITIONS
 ****************************************************************************************/

namespace bounds {
    static const double COG_LOWER = -5.0; //meter
    static const double COG_UPPER = 5.0; //meter
    static const double LENGTH_LOWER = -5.0; //meter
    static const double LENGTH_UPPER = 5.0; //meter
    static const double TENSOR_LOWER = -1.0;
    static const double TENSOR_UPPER = 1.0;
    static const double MASS_LOWER = 0.0; //Kg
    static const double MASS_UPPER = 5.0; //Kg
    static const double ANGLE_LOWER = -180.0; //deg
    static const double ANGLE_UPPER = 180.0; //deg
};

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

//PARAMETER TYPES

//Center of gravity parameter type
class center_of_gravity_t
{
public:
    Eigen::Vector3d value;

    center_of_gravity_t(): value(Eigen::Vector3d::Zero()){}

    void set(const Eigen::Vector3d val) {value = val;}
};

//Inertia tensor parameter type
class tensor_t
{
public:
    Eigen::MatrixXd value;

    tensor_t(){};

    void set(const Eigen::MatrixXd val) {value = val;}
};

//Length (DH table) type - for matrices
class length_matrix_t
{
public:
    Eigen::MatrixXd value;

    length_matrix_t(){};
    void set(const Eigen::MatrixXd val) {value = val;}
};

//Length type
class length_t
{
public:
    double value;

    length_t(): value(0){}
    void set(const double val) {value = val;}
};

//Mass parameter type
class mass_t
{
public:
    double value;

    mass_t(): value(0){}
    void set(const double val) {value = val;}
};

//Sign parameter type
class sign_t
{
public:
    int value;

    sign_t(): value(0){}
    void set(const double val) {value = val/abs(val);}
};

//Angle parameter type
class angle_t
{
public:
    double value;

    angle_t(): value(0){}
    void set(const double val) {value = val;}
};

//Angle parameter type - for matrices
class angle_matrix_t
{
public:
    Eigen::MatrixXd value;

    angle_matrix_t(){}
    void set(const Eigen::MatrixXd val) {value = val;}
};

//Selector parameter type
class selector_t
{
public:
    bool value;

    selector_t(): value(0){}
    void set(const double val) {value = (val!=0);}
};


//Primitive types (using _<type> for a primitive type)
class _int
{
public:
    int value;

    _int(): value(0){}
    void set(const double val) {value = static_cast<int>(val);}
};

class _float
{
public:
    float value;

    _float(): value(0){}
    void set(const double val) {value = static_cast<float>(val);}
};

class _double
{
public:
    double value;

    _double(): value(0){}
    void set(const double val) {value = val;}
};

class _uint16_t
{
public:
    uint16_t value;

    _uint16_t(): value(0){}
    void set(const double val) {value = static_cast<uint16_t>(val);}
};

class _int16_t
{
public:
    int16_t value;

    _int16_t(): value(0){}
    void set(const double val) {value = static_cast<int16_t>(val);}
};

class _Vector3d
{
public:
    Eigen::Vector3d value;

    _Vector3d(): value(Eigen::Vector3d::Zero()){}
    void set(const Eigen::Vector3d val) {value = val;}
};

class _Matrix
{
public:
    Eigen::MatrixXd value;

    _Matrix(){}
    void set(const Eigen::MatrixXd val) {value = val;}
};

template<class T>
struct is_length_type {
    enum { value = 0 };
};

template<>
struct is_length_type<length_t> {
    enum { value = 1 };
};


template<class T>
struct is_mass_type {
    enum { value = 0 };
};

template<>
struct is_mass_type<mass_t> {
    enum { value = 1 };
};


template<class T>
struct is_angle_type {
    enum { value = 0 };
};

template<>
struct is_angle_type<angle_t> {
    enum { value = 1 };
};

template<class T>
struct is_sign_type {
    enum { value = 0 };
};

template<>
struct is_sign_type<sign_t> {
    enum { value = 1 };
};

template<class T>
struct is_selector_type {
    enum { value = 0 };
};

template<>
struct is_selector_type<selector_t> {
    enum { value = 1 };
};


template<class T>
struct is_center_of_gravity_type {
    enum { value = 0 };
};

template<>
struct is_center_of_gravity_type<center_of_gravity_t> {
    enum { value = 1 };
};


template<class T>
struct is_tensor_type {
    enum { value = 0 };
};

template<>
struct is_tensor_type<tensor_t> {
    enum { value = 1 };
};


template<class T>
struct is_angle_matrix_type {
    enum { value = 0 };
};

template<>
struct is_angle_matrix_type<angle_matrix_t> {
    enum { value = 1 };
};


template<class T>
struct is_length_matrix_type {
    enum { value = 0 };
};

template<>
struct is_length_matrix_type<length_matrix_t> {
    enum { value = 1 };
};

#endif // DATA_TYPES_HANDLING_H
