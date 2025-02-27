/*! \file DiagnosticNodeProcess.h
 */
#ifndef DiagnosticNodeProcess_H
#define DiagnosticNodeProcess_H
#include <eros/BaseNodeProcess.h>
namespace eros_nodes::Infrastructure {
/*! \class DiagnosticNodeProcess DiagnosticNodeProcess.h "DiagnosticNodeProcess.h"
 *  \brief The process utility for the DiagnosticNode. */
class DiagnosticNodeProcess : public eros::BaseNodeProcess
{
   public:
    DiagnosticNodeProcess();
    ~DiagnosticNodeProcess();
    // Constants

    // Enums

    // Structs
    /*! \struct DiagnosticTypeAggregate
        \brief DiagnosticTypeAggregate Container
        Container to aggregate Diagnostic Information.
    */
    struct DiagnosticTypeAggregate {
        DiagnosticTypeAggregate() : update_count(0) {
        }
        eros::eros_diagnostic::Diagnostic worst_diag;
        std::string worst_diag_key;
        uint64_t update_count;
        std::map<std::string, eros::eros_diagnostic::Diagnostic> diag_list;
    };

    // Initialization Functions
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions
    eros::eros_diagnostic::Diagnostic get_worst_diagnostic(
        eros::eros_diagnostic::DiagnosticType type);

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    eros::eros_diagnostic::Diagnostic update_topiclist(
        std::vector<std::string>& new_diagnostic_topics_to_subscribe);

    // Message Functions
    bool new_external_diagnostic(eros::eros_diagnostic::Diagnostic diag);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);

    // Destructors
    void cleanup() {
        base_cleanup();
        return;
    }

    // Printing Functions
    std::string pretty() override;

   private:
    std::string build_key(eros::eros_diagnostic::Diagnostic);
    std::vector<std::string> diagnostic_topics_already_monitoring;
    std::map<eros::eros_diagnostic::DiagnosticType, DiagnosticTypeAggregate> diagnostic_aggregator;
};
}  // namespace eros_nodes::Infrastructure
#endif  // DiagnosticNodeProcess_H