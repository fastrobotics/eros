#include <eros_diagnostic/DiagnosticManager.h>
#include <eros_diagnostic/DiagnosticUtility.h>
namespace eros::eros_diagnostic {
std::string DiagnosticManager::pretty(std::string pre, std::vector<Diagnostic> diagnostics) {
    std::string str = "";
    if (diagnostics.size() == 0) {
        str = pre + "NO DIAGNOSTICS DEFINED YET.";
        return str;
    }
    for (std::size_t i = 0; i < diagnostics.size(); ++i) {
        str += "Diag: \n" + DiagnosticUtility::pretty("\t", diagnostics.at(i)) + "\n";
    }
    return str;
}
bool DiagnosticManager::enable_diagnostics(std::vector<DiagnosticType> diagnostic_types) {
    if (initialized == false) {
        return false;
    }
    std::sort(diagnostic_types.begin(), diagnostic_types.end());
    for (std::size_t i = 0; i < diagnostic_types.size(); ++i) {
        Diagnostic diag = root_diagnostic;
        diag.type = diagnostic_types.at(i);
        /*
        if (diag.type == DiagnosticType::SYSTEM_RESOURCE)  // This is special, so we don't throw
                                                           // a ton of warn messages when the
                                                           // system launches.
        {
            diag.level = Level::Type::NOTICE;
            diag.message = Message::INITIALIZING;
            diag.description = "Initializing Resource Monitor.";
        }
        else {
            diag.level = Level::Type::WARN;
            diag.message = Message::INITIALIZING;
            diag.description = "Initializing Diagnostic.";
        }
        */
        bool add_me = true;
        for (std::size_t j = 0; j < diagnostics.size(); ++j) {
            if (diagnostics.at(j).type == diagnostic_types.at(i)) {
                add_me = false;
            }
        }
        if (add_me == true) {
            diagnostics.push_back(diag);
        }
    }
    return true;
}
std::vector<Diagnostic> DiagnosticManager::get_latest_diagnostics() {
    std::vector<Diagnostic> latest_diagnostics;
    for (std::size_t i = 0; i < diagnostics.size(); ++i) {
        {
            if (diagnostics.at(i).update_count > 0) {
                latest_diagnostics.push_back(diagnostics.at(i));
            }
            diagnostics.at(i).update_count = 0;
        }
    }
    return latest_diagnostics;
}
Diagnostic DiagnosticManager::read_diagnostic(DiagnosticType diagnostic_type) {
    for (std::size_t i = 0; i < diagnostics.size(); ++i) {
        if (diagnostic_type == diagnostics.at(i).type) {
            return diagnostics.at(i);
        }
    }
    Diagnostic not_found;
    return not_found;
}
//! Update Diagnostic
/*!
    \brief Updates an enabled diagnostic with the provided diagnostic.
*/
Diagnostic DiagnosticManager::update_diagnostic(Diagnostic diag) {
    return update_diagnostic(
        diag.device_name, diag.type, diag.level, diag.message, diag.description);
}
Diagnostic DiagnosticManager::update_diagnostic(DiagnosticType diagnostic_type,
                                                Level::Type level,
                                                Message message,
                                                std::string description) {
    return update_diagnostic(
        root_diagnostic.device_name, diagnostic_type, level, message, description);
}

Diagnostic DiagnosticManager::update_diagnostic(std::string device_name,
                                                DiagnosticType diagnostic_type,
                                                Level::Type level,
                                                Message message,
                                                std::string description) {
    bool devicetype_found = false;
    bool devicename_found = false;
    Diagnostic diag;
    uint8_t insert_index = -1;
    for (std::size_t i = 0; i < diagnostics.size(); ++i) {
        if (diagnostic_type == diagnostics.at(i).type) {
            devicetype_found = true;
            insert_index = i;
            if (diagnostics.at(i).device_name == device_name) {
                devicename_found = true;
                diag = diagnostics.at(i);
                diag.level = level;
                diag.message = message;
                diag.description = description;
                diag.update_count++;
                diagnostics.at(i) = diag;
            }
        }
    }
    if ((devicetype_found == true) and (devicename_found == false)) {
        diag = root_diagnostic;
        diag.type = diagnostic_type;
        diag.device_name = device_name;
        diag.level = level;
        diag.message = message;
        diag.description = description;
        diag.update_count++;
        std::vector<Diagnostic>::iterator it;
        it = diagnostics.begin();
        diagnostics.insert(it + insert_index, diag);
    }
    if (devicetype_found == true) {
        return diag;
    }
    else {
        diag = root_diagnostic;
        diag.type = diagnostic_type;
        diag.level = Level::Type::ERROR;
        diag.message = Message::INITIALIZING_ERROR;
        char tempstr[512];
        sprintf(tempstr,
                "Unsupported Diagnostic Type: %s(%d).  Did you forget to enable it?",
                DiagnosticUtility::DiagnosticTypeString(diagnostic_type).c_str(),
                (uint8_t)diagnostic_type);
        diag.description = std::string(tempstr);
        return diag;
    }
}

}  // namespace eros::eros_diagnostic