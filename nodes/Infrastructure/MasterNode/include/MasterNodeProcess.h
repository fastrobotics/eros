/*! \file MasterNodeProcess.h
 */
#ifndef MasterNodeProcess_H
#define MasterNodeProcess_H
#include <eros/BaseNodeProcess.h>
namespace eros_nodes::Infrastructure {
/*! \class MasterNodeProcess MasterNodeProcess.h "MasterNodeProcess.h"
 *  \brief The process utility for the Master Node. */
class MasterNodeProcess : public eros::BaseNodeProcess
{
   public:
    MasterNodeProcess();
    ~MasterNodeProcess();
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
}  // namespace eros_nodes::Infrastructure
#endif  // MasterNodeProcess_H
