#include <eros_window/DiagnosticsWindow/DiagnosticsWindow.h>
namespace eros_window {
constexpr double DiagnosticsWindow::START_X_PERC;
constexpr double DiagnosticsWindow::START_Y_PERC;
constexpr double DiagnosticsWindow::WIDTH_PERC;
constexpr double DiagnosticsWindow::HEIGHT_PERC;
DiagnosticsWindow::~DiagnosticsWindow() {
}

bool DiagnosticsWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }

    request_data_timer += dt;
    if (request_data_timer >= REQUEST_DATA_SEC) {
        request_data_timer = 0.0;
        if (diagnostic_mode == DiagnosticMode::SYSTEM) {
            std::string system_diagnostic_topic = robot_namespace + "srv_system_diagnostics";
            ros::ServiceClient system_diag_client =
                nodeHandle->serviceClient<eros::srv_get_diagnostics>(system_diagnostic_topic);
            eros::srv_get_diagnostics srv;
            srv.request.MinLevel = 0;
            srv.request.DiagnosticType = 0;
            if (system_diag_client.call(srv)) {
                for (auto eros_diag : srv.response.diag_list) {
                    eros::eros_diagnostic::Diagnostic diag =
                        eros::eros_diagnostic::DiagnosticUtility::convert(eros_diag);
                    diagnostic_data[(uint8_t)diag.type] = diag;
                }
            }
            else {
                logger->log_warn("Unable to request System Diagnostics.");
            }
        }
        else if (diagnostic_mode == DiagnosticMode::NODE) {
            std::string node_diagnostic_topic = node_to_monitor + "/srv_diagnostics";
            ros::ServiceClient diag_client =
                nodeHandle->serviceClient<eros::srv_get_diagnostics>(node_diagnostic_topic);
            eros::srv_get_diagnostics srv;
            srv.request.MinLevel = 0;
            srv.request.DiagnosticType = 0;

            if (diag_client.call(srv)) {
                for (auto eros_diag : srv.response.diag_list) {
                    eros::eros_diagnostic::Diagnostic diag =
                        eros::eros_diagnostic::DiagnosticUtility::convert(eros_diag);
                    diagnostic_data[(uint8_t)diag.type] = diag;
                }
            }
            else {
                logger->log_warn("Unable to request Node Diagnostics.");
            }
        }
    }
    status = update_window();
    return status;
}
bool DiagnosticsWindow::new_command(std::vector<eros_window::WindowCommand> commands) {
    for (auto command : commands) {
        if (command.type == eros_window::WindowCommandType::VIEW_DIAGNOSTICS_NODE) {
            diagnostic_data.clear();
            diagnostic_mode = DiagnosticMode::NODE;
            node_to_monitor = command.option;
            return true;
        }
        else if (command.type == eros_window::WindowCommandType::VIEW_DIAGNOSTICS_SYSTEM) {
            diagnostic_data.clear();
            diagnostic_mode = DiagnosticMode::SYSTEM;
            node_to_monitor = "";
            return true;
        }
    }
    return true;
}
bool DiagnosticsWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    // GCOVR_EXCL_START
    uint16_t index = 0;
    std::string header_string;
    if (diagnostic_mode == DiagnosticMode::SYSTEM) {
        header_string = "System Diagnostics:";
    }
    else if (diagnostic_mode == DiagnosticMode::NODE) {
        header_string = "Node: " + node_to_monitor + " Diagnostics:";
    }
    mvwprintw(get_window(), 1, 1, header_string.c_str());
    wclrtoeol(get_window());
    std::string dashed(get_screen_coordinates_pixel().width_pix - 2, '-');
    mvwprintw(get_window(), 2, 1, dashed.c_str());
    for (auto& diag : diagnostic_data) {
        eros_window::Color color;
        std::string str =
            "  " + eros::eros_diagnostic::DiagnosticUtility::DiagnosticTypeString(diag.second.type);
        if (diag.second.message == eros::eros_diagnostic::Message::NODATA) {
            color = eros_window::Color::NO_COLOR;
        }
        else {
            switch (diag.second.level) {
                case eros::Level::Type::DEBUG: color = eros_window::Color::NO_COLOR; break;
                case eros::Level::Type::INFO: color = eros_window::Color::GREEN_COLOR; break;
                case eros::Level::Type::NOTICE: color = eros_window::Color::GREEN_COLOR; break;
                case eros::Level::Type::WARN:
                    color = eros_window::Color::YELLOW_COLOR;
                    str += ": " + diag.second.description;
                    break;
                case eros::Level::Type::ERROR:
                    color = eros_window::Color::RED_COLOR;
                    str += ": " + diag.second.description;
                    break;
                case eros::Level::Type::FATAL:
                    color = eros_window::Color::RED_COLOR;
                    str += ": " + diag.second.description;
                    break;
                default: color = eros_window::Color::RED_COLOR; break;
            }
        }

        str += "  ";
        if (str.size() > (std::size_t)(get_screen_coordinates_pixel().width_pix - 4)) {
            str =
                str.substr(0, (std::size_t)(get_screen_coordinates_pixel().width_pix - 4)) + "...";
        }
        wattron(get_window(), COLOR_PAIR(color));
        mvwprintw(get_window(), index + 3, 1, str.c_str());
        wclrtoeol(get_window());
        wattroff(get_window(), COLOR_PAIR(color));
        index++;
    }
    wclrtobot(get_window());
    if (focused) {
        box(get_window(), '.', '.');
    }
    else {
        box(get_window(), 0, 0);
    }
    wrefresh(get_window());
    return true;
    // GCOVR_EXCL_STOP
}
}  // namespace eros_window