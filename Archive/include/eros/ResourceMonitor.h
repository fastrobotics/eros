/*! \file ResourceMonitor.h
 */
#pragma once
#include <eros/Logger.h>
#include <eros/eros_Definitions.h>
#include <eros/loadfactor.h>
#include <eros_diagnostic/Diagnostic.h>
#include <eros_utility/CoreUtility.h>
#include <stdio.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include "ros/time.h"
namespace eros {
/*! \class ResourceMonitor
    \brief ResourceMonitor class
    ResourceMonitor class used to collect resource information on a process or device.
*/
class ResourceMonitor
{
   public:
    enum class Mode {
        UNKNOWN = 0, /*!< Uninitialized value. */
        PROCESS = 1, /*!< This Mode is used when checking Process Resource Usage Information. */
        DEVICE =
            2, /*!< This Mode is used when checking Device Resource Availability Information. */
        END_OF_LIST = 3 /*!< Last item of list. Used for Range Checks. */
    };
    ~ResourceMonitor();
    ResourceMonitor(std::string name,
                    Mode _mode,
                    eros_diagnostic::Diagnostic _diag,
                    Logger* _logger);
    bool is_initialized() {
        return initialized;
    }

    std::string pretty(ResourceInfo info);
    eros_diagnostic::Diagnostic init();
    ResourceInfo get_resourceinfo() {
        return resourceInfo;
    }
    Architecture::Type get_architecture() {
        return architecture;
    }
    eros_diagnostic::Diagnostic update(double t_dt);
    eros_diagnostic::Diagnostic get_latest_diagnostic() {
        return diagnostic;
    }
    eros::loadfactor get_load_factor() {
        return load_factor;
    }

    bool reset();

   private:
    eros_diagnostic::Diagnostic read_process_resource_usage();
    eros_diagnostic::Diagnostic read_device_resource_availability();
    eros_diagnostic::Diagnostic read_device_loadfactor();
    Architecture::Type read_device_architecture();

    Mode mode;
    Architecture::Type architecture;
    eros_diagnostic::Diagnostic diagnostic;
    eros::loadfactor load_factor;
    Logger* logger;
    ResourceInfo resourceInfo;
    bool initialized;
    double run_time;
    uint16_t processor_count;
};
}  // namespace eros