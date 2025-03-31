/*! \file {{cookiecutter.class_name}}.h
 */
#pragma once
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// ROS Services
// ROS Actions
// Project Includes
#include <eros/Logger.h>
/**
 * @brief {{cookiecutter.package_name}} Namespace
 *
 */
namespace {{cookiecutter.package_name}} {
/*! \class {{cookiecutter.class_name}}
    \brief {{cookiecutter.class_name}}
    A description of your new Class.
*/
class {{cookiecutter.class_name}}
{
   public:
    // Constructors & Deconstructors
    {{cookiecutter.class_name}}() : logger(nullptr), run_time(0.0) {
    }
    virtual ~{{cookiecutter.class_name}}() {
    }
    // Constants

    // Enums
    enum class Enum1 {
        UNKNOWN = 0,    /*!< Uninitialized value. */
        VALUE1 = 1,     /*!< Last item of list. Used for Range Checks. */
        END_OF_LIST = 2 /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert {{cookiecutter.class_name}}::Enum1 to human readable string
    /*!
      \param v {{cookiecutter.class_name}}::Enum1 type
      \return The converted string.
    */
    static std::string Enum1String({{cookiecutter.class_name}}::Enum1 v) {
        switch (v) {
            case {{cookiecutter.class_name}}::Enum1::UNKNOWN: return "UNKNOWN"; break;
            case {{cookiecutter.class_name}}::Enum1::VALUE1: return "VALUE1"; break;
            default: return Enum1String({{cookiecutter.class_name}}::Enum1::UNKNOWN); break;
        }
    }
    //! Convert human readable string to {{cookiecutter.class_name}}::Enum1
    /*!
      \param v Human Readable String
      \return {{cookiecutter.class_name}}::Enum1 type
    */
    static {{cookiecutter.class_name}}::Enum1 Enum1Type(std::string v) {
        if (v == "VALUE1") {
            return {{cookiecutter.class_name}}::Enum1::VALUE1;
        }
        else {
            return {{cookiecutter.class_name}}::Enum1::UNKNOWN;
        }
    }

    // Structs
    /*! \struct Struct1
        \brief Struct1 Information:
        Holds information about Struct1.
    */
    struct Struct1 {
        /*@{*/
        uint16_t Param1;    /**< Parmam1. */
        std::string Param2; /**< Param2. */
        /*@}*/
    };

    // Initializing & Reset Functions
    /*! \brief Initialize Object. */
    bool init(eros::Logger* logger);
    /*! \brief Reset Object */
    bool reset();

    // Update Functions
    /*! \brief Update objet at some interval. */
    bool update(double dt);

    // Attribute Functions
    double get_runtime() {
        return run_time;
    }

    // Message Functions

    // Support Functions

    // Printing Functions
    std::string pretty();

    // Finish Functions
    bool finish();

   private:
    eros::Logger* logger;
    double run_time;
};

}  // namespace {{cookiecutter.package_name}}