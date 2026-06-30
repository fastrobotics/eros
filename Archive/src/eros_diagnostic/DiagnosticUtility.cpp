#include <eros_diagnostic/DiagnosticUtility.h>
namespace eros::eros_diagnostic {

eros::diagnostic DiagnosticUtility::convert(Diagnostic diag) {
    eros::diagnostic diag_msg;
    diag_msg.DeviceName = diag.device_name;
    diag_msg.NodeName = diag.node_name;
    diag_msg.System = (uint8_t)diag.system;
    diag_msg.SubSystem = (uint8_t)diag.subsystem;
    diag_msg.Component = (uint8_t)diag.component;
    diag_msg.DiagnosticType = (uint8_t)diag.type;
    diag_msg.DiagnosticMessage = (uint8_t)diag.message;
    diag_msg.Level = (uint8_t)diag.level;
    diag_msg.Description = diag.description;
    return diag_msg;
}

Diagnostic DiagnosticUtility::convert(eros::diagnostic diag_msg) {
    Diagnostic diag;
    diag.device_name = diag_msg.DeviceName;
    diag.node_name = diag_msg.NodeName;
    diag.system = (System::MainSystem)diag_msg.System;
    diag.subsystem = (System::SubSystem)diag_msg.SubSystem;
    diag.component = (System::Component)diag_msg.Component;
    diag.type = (DiagnosticType)diag_msg.DiagnosticType;
    diag.message = (Message)diag_msg.DiagnosticMessage;
    diag.level = (Level::Type)diag_msg.Level;
    diag.description = diag_msg.Description;
    return diag;
}
std::string DiagnosticUtility::DiagnosticTypeString(DiagnosticType v) {
    switch (v) {
        case DiagnosticType::UNKNOWN: return "UNKNOWN"; break;
        case DiagnosticType::UNKNOWN_TYPE: return "UNKNOWN_TYPE"; break;
        case DiagnosticType::SOFTWARE: return "SOFTWARE"; break;
        case DiagnosticType::COMMUNICATIONS: return "COMMUNICATIONS"; break;
        case DiagnosticType::SENSORS: return "SENSORS"; break;
        case DiagnosticType::ACTUATORS: return "ACTUATORS"; break;
        case DiagnosticType::DATA_STORAGE: return "DATA_STORAGE"; break;
        case DiagnosticType::REMOTE_CONTROL: return "REMOTE_CONTROL"; break;
        case DiagnosticType::TARGETING: return "TARGETING"; break;
        case DiagnosticType::POSE: return "POSE"; break;
        case DiagnosticType::TIMING: return "TIMING"; break;
        case DiagnosticType::SYSTEM_RESOURCE: return "SYSTEM_RESOURCE"; break;
        default: return DiagnosticTypeString(DiagnosticType::UNKNOWN); break;
    }
}
std::string DiagnosticUtility::DiagnosticMessageString(Message v) {
    switch (v) {
        case Message::UNKNOWN: return "UNKNOWN"; break;
        case Message::NOERROR: return "NOERROR"; break;
        case Message::NODATA: return "NODATA"; break;
        case Message::UNKNOWN_ERROR: return "UNKNOWN_ERROR"; break;
        case Message::INITIALIZING: return "INITIALIZING"; break;
        case Message::INITIALIZING_ERROR: return "INITIALIZING_ERROR"; break;
        case Message::DROPPING_PACKETS: return "DROPPING_PACKETS"; break;
        case Message::MISSING_HEARTBEATS: return "MISSING_HEARTBEATS"; break;
        case Message::DEVICE_NOT_AVAILABLE: return "DEVICE_NOT_AVAILABLE"; break;
        case Message::TEMPERATURE_HIGH: return "TEMPERATURE_HIGH"; break;
        case Message::TEMPERATURE_LOW: return "TEMPERATURE_LOW"; break;
        case Message::RESOURCE_LEAK: return "RESOURCE_LEAK"; break;
        case Message::HIGH_RESOURCE_USAGE: return "HIGH_RESOURCE_USAGE"; break;
        case Message::DIAGNOSTIC_FAILED: return "DIAGNOSTIC_FAILED"; break;
        default: return DiagnosticMessageString(Message::UNKNOWN); break;
    }
}

std::string DiagnosticUtility::pretty(std::string pre, Diagnostic diag, bool print_end_line) {
    std::string str = "";
    str += pre + "Level: " + Level::LevelString(diag.level);
    print_end_line ? str += "\n" : str += " ";
    str += pre + "Device: " + diag.device_name;
    print_end_line ? str += "\n" : str += " ";
    str += pre + "Node: " + diag.node_name;
    print_end_line ? str += "\n" : str += " ";
    str += pre + "System: " + System::MainSystemString(diag.system);
    print_end_line ? str += "\n" : str += " ";
    str += pre + "Subsystem: " + System::SubSystemString(diag.subsystem);
    print_end_line ? str += "\n" : str += " ";
    str += pre + "Component: " + System::ComponentString(diag.component);
    print_end_line ? str += "\n" : str += " ";
    str += pre + "Diagnostic Type: " + DiagnosticTypeString(diag.type);
    print_end_line ? str += "\n" : str += " ";
    str += pre + "Message: " + DiagnosticMessageString(diag.message);
    print_end_line ? str += "\n" : str += " ";
    str += pre + "Desc: " + diag.description;
    return str;
}
}  // namespace eros::eros_diagnostic