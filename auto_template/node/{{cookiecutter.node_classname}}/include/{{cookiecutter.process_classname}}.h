/*! \file {{cookiecutter.process_classname}}.h
 */
#pragma once
#include <eros/BaseNodeProcess.h>
namespace {{cookiecutter.package_name}} {
/*! \class {{cookiecutter.process_classname}} {{cookiecutter.process_classname}}.h "{{cookiecutter.process_classname}}.h"
 *  \brief */
class {{cookiecutter.process_classname}} : public eros::BaseNodeProcess
{
   public:
    {{cookiecutter.process_classname}}();
    ~{{cookiecutter.process_classname}}();
    // Constants

    // Enums

    // Structs

    // Initialization Functions
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);

    // Destructors
    void cleanup() {
        base_cleanup();
        return;
    }

    // Printing Functions
    std::string pretty() override;

   private:
};
}  // namespace {{cookiecutter.package_name}}
