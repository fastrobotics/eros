/*! \file Logger.h
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <eros_diagnostic/Diagnostic.h>
#include <stdio.h>

#include <ctime>
#include <fstream>
#include <iostream>

#include "eros_Definitions.h"

#ifdef ROS_INSTALLED
#include <ros/console.h>
#endif
namespace eros {

//! Log a Debug Line
/*!
  \param tempstr The string to output.
*/
#define log_debug(tempstr) LOG_DEBUG(__FILE__, __LINE__, tempstr)

//! Log a Info Line
/*!
  \param tempstr The string to output.
*/
#define log_info(tempstr) LOG_INFO(__FILE__, __LINE__, tempstr)

//! Log a Notice Line
/*!
  \param tempstr The string to output.
*/
#define log_notice(tempstr) LOG_NOTICE(__FILE__, __LINE__, tempstr)

//! Log a Warn Line
/*!
  \param tempstr The string to output.
*/
#define log_warn(tempstr) LOG_WARN(__FILE__, __LINE__, tempstr)

//! Log a Error Line
/*!
  \param tempstr The string to output.
*/
#define log_error(tempstr) LOG_ERROR(__FILE__, __LINE__, tempstr)

//! Log a Fatal Line
/*!
  \param tempstr The string to output.
*/
#define log_fatal(tempstr) LOG_FATAL(__FILE__, __LINE__, tempstr)

//! Log a Diagnostic
/*!
  \param diagnostic The diagnostic to output.
*/
#define log_diagnostic(diagnostic) LOG_DIAGNOSTIC(__FILE__, __LINE__, diagnostic)

/*! \class Logger
    \brief Logger class
    Logger class used to write log outputs for strings and diagnostic information to console and
   output text files.
*/
class Logger
{
   public:
    static constexpr uint16_t MAXLINE_COUNT = 5000;
    enum class LoggerStatus {
        UNKNOWN = 0,        /*!< Uninitialized value. */
        FAILED_TO_OPEN = 1, /*!< Logger was not able to open file to log. */
        LOG_WRITTEN = 2,    /*!< Log file was updated. */
        LOG_SUPPRESSED = 3, /*!< Log entry was suppressed. */
        END_OF_LIST = 4     /*!< Last item of list. Used for Range Checks. */
    };
    Logger();
    ~Logger();
    //! Instantiate a Logger
    /*!
      \param level A string representation of the Verbosity Level.  Possible values:
      DEBUG,INFO,NOTICE,WARN,ERROR,FATAL \param name The instance name of the log file.
    */
    Logger(std::string level, std::string name);
    //! Instantiate a Logger
    /*!
      \param level A string representation of the Verbosity Level.  Possible values:
      DEBUG,INFO,NOTICE,WARN,ERROR,FATAL \param directory Where to put the log output file. \param
      name The instance name of the log file.
    */
    Logger(std::string level, std::string directory, std::string name);
    Level::Type get_logverbosity() {
        return verbosity;
    }
    bool is_logger_ok() {
        return logger_ok;
    }
    /*!
      \param level The new verbosity.
      \return If the verbosity was able to be set (True) or not (False)
    */
    bool set_logverbosity(Level::Type level);

    //! Disable Console Output
    /*!
      \brief Does not affect ROS Console Output
    */
    void disable_consoleprint() {
        log_warn("Disabling Console Print");
        console_print = false;
    }
    void enable_consoleprint() {
        log_notice("Enabling Console Print");
        console_print = true;
    }

    //! Enable ROS Logger
    /*!
      \return If the ROS Console Output was enabled or not.
    */
    bool enable_ROS_logger() {
#ifdef ROS_INSTALLED
        use_ROS_logger = true;
        return true;
#else
        use_ROS_logger = false;
#endif
        return false;
    }

    //! Log a Debug Line.  Do not use, use: log_debug
    LoggerStatus LOG_DEBUG(std::string filename, uint64_t linenumber, std::string tempstr);
    //! Log a Info Line.  Do not use, use: log_info
    LoggerStatus LOG_INFO(std::string filename, uint64_t linenumber, std::string tempstr);
    //! Log a Notice Line.  Do not use, use: log_notice
    LoggerStatus LOG_NOTICE(std::string filename, uint64_t linenumber, std::string tempstr);
    //! Log a Warn Line.  Do not use, use: log_warn
    LoggerStatus LOG_WARN(std::string filename, uint64_t linenumber, std::string tempstr);
    //! Log a Error Line.  Do not use, use: log_error
    LoggerStatus LOG_ERROR(std::string filename, uint64_t linenumber, std::string tempstr);
    //! Log a Fatal Line.  Do not use, use: log_fatal
    LoggerStatus LOG_FATAL(std::string filename, uint64_t linenumber, std::string tempstr);
    //! Log a Diagnostic.  Do not use, use: log_diagnostic
    LoggerStatus LOG_DIAGNOSTIC(std::string filename,
                                uint64_t linenumber,
                                eros_diagnostic::Diagnostic diagnostic);

   private:
    const std::string GREEN_FOREGROUND = "\033[1;32m";
    const std::string YELLOW_FOREGROUND = "\033[1;33m";
    const std::string RED_FOREGROUND = "\033[1;31m";
    const std::string END_COLOR = "\033[0m";

    bool logger_ok;
    int line_counter;
    Level::Type verbosity;
    std::ofstream log_file;
    std::string node_name;
    char file_path[120];
    std::string map_logverbosity_tostring(int v);
    int map_logverbosity_toint(std::string level);
    LoggerStatus print_log(std::string filename,
                           uint64_t linenumber,
                           Level::Type level,
                           std::string tempstr);
    bool console_print{true};
    bool use_ROS_logger{false};
};
}  // namespace eros
#endif  // LOGGER_H