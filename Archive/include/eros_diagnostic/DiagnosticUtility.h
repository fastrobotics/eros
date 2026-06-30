#pragma once
#include <eros/diagnostic.h>
#include <eros_diagnostic/Diagnostic.h>
namespace eros::eros_diagnostic {
class DiagnosticUtility
{
   public:
    static eros::diagnostic convert(Diagnostic diagnostic);
    static Diagnostic convert(eros::diagnostic diag);
    //! Convert DiagnosticType to human readable string
    /*!
      \param v DiagnosticType type
      \return The converted string.
    */
    static std::string DiagnosticTypeString(DiagnosticType v);
    //! Convert Message to human readable string
    /*!
      \param v Message type
      \return The converted string.
    */
    static std::string DiagnosticMessageString(Message v);

    //! Generate human readable string of a diagnostic
    /*!
        \param pre A string to be pre-appended to the human readable string.
        \param diag A diagnostic.
        \return Human readable string of diagnostic.
    */
    static std::string pretty(std::string pre, Diagnostic diag, bool print_end_line = true);
};

}  // namespace eros::eros_diagnostic