#pragma once
#include <eros/eros_Definitions.h>

#include "Diagnostic.h"

namespace eros::eros_diagnostic {
class DiagnosticManager
{
   public:
    DiagnosticManager() {
    }
    virtual ~DiagnosticManager() {
    }
    //! Generate human readable string of all enabled diagnostics.
    /*!
        \return Human readable string of all enabled diagnostics.
    */
    std::string pretty() {
        return pretty("", diagnostics);
    }
    void initialize(Diagnostic diag) {
        root_diagnostic = diag;
        initialized = true;
    }
    //! Enable Diagnostic Types
    /*!
        \param diagnostic_types A vector of Diagnostic Types that should be enabled.
        \return If the diagnostics were enabled or not.
    */
    bool enable_diagnostics(std::vector<DiagnosticType> diagnostic_types);
    Diagnostic get_root_diagnostic() {
        return root_diagnostic;
    }
    std::vector<Diagnostic> get_diagnostics() {
        return diagnostics;
    }
    std::vector<Diagnostic> get_latest_diagnostics();
    /**
     * @brief Read Diagnostic by Type
     *
     * @param diagnostic_type
     * @return Diagnostic
     */
    Diagnostic read_diagnostic(DiagnosticType diagnostic_type);
    //! Update Diagnostic
    /*!
        \brief Updates an enabled diagnostic with the provided diagnostic.
    */
    Diagnostic update_diagnostic(Diagnostic diag);
    //! Update Diagnostic
    /*!
        \brief Updates an enabled diagnostic with the defined information.
    */
    Diagnostic update_diagnostic(DiagnosticType diagnostic_type,
                                 Level::Type level,
                                 Message message,
                                 std::string description);

    //! Update Diagnostic
    /*!
        \brief Updates an enabled diagnostic with the fully defined diagnostic information.
    */
    Diagnostic update_diagnostic(std::string device_name,
                                 DiagnosticType diagnostic_type,
                                 Level::Type level,
                                 Message message,
                                 std::string description);

   private:
    //! Generate human readable string of a vector of diagnostics
    /*!
        \param pre A string to be pre-appended to the human readable string.
        \param diagnostics A vector of diagnostics.
        \return Human readable string of diagnostics.
    */
    static std::string pretty(std::string pre, std::vector<Diagnostic> diagnostics);
    bool initialized{false};
    Diagnostic root_diagnostic;
    std::vector<Diagnostic> diagnostics;
};
}  // namespace eros::eros_diagnostic