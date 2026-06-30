#include "NodeWindow/NodeWindow.h"
namespace eros_nodes::SystemMonitor {
constexpr double NodeWindow::START_X_PERC;
constexpr double NodeWindow::START_Y_PERC;
constexpr double NodeWindow::WIDTH_PERC;
constexpr double NodeWindow::HEIGHT_PERC;

NodeWindow::~NodeWindow() {
}
std::string NodeWindow::get_nodeheader() {
    std::string str = "";
    std::map<NodeFieldColumn, eros_window::Field>::iterator it = node_window_fields.begin();
    while (it != node_window_fields.end()) {
        // Check if field name is too long:
        if (it->second.text.size() > it->second.width) {
            str += it->second.text.substr(0, it->second.width);
        }
        else {
            str += it->second.text;
            // Figure out how many spaces to add
            std::size_t spaces = it->second.width - it->second.text.size();
            for (std::size_t j = 0; j < spaces; ++j) { str += " "; }
        }
        ++it;
    }

    if (str.size() > mainwindow_width) {
        logger->enable_consoleprint();
        logger->log_warn("Node Header too long for Window!.");
        return "";
    }
    return str;
}
bool NodeWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }

    std::vector<NodeData>::iterator node_it = node_list.begin();
    while (node_it != node_list.end()) {
        node_it->last_heartbeat_delta += dt;
        if (node_it->last_heartbeat_delta >= eros_window::COMMTIMEOUT_THRESHOLD) {
            node_it->state = eros::Node::State::UNKNOWN;
        }
        ++node_it;
    }
    status = update_window();
    return status;
}
bool NodeWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    // GCOVR_EXCL_START
    const uint16_t TASKSTART_COORD_Y = 1;
    const uint16_t TASKSTART_COORD_X = 1;
    uint16_t index = 0;
    std::vector<NodeData>::iterator node_it;
    for (node_it = node_list.begin(); node_it != node_list.end(); node_it++) {
        eros_window::Color color = eros_window::Color::UNKNOWN;
        switch (node_it->state) {
            case eros::Node::State::UNKNOWN: color = eros_window::Color::RED_COLOR; break;
            case eros::Node::State::START: color = eros_window::Color::YELLOW_COLOR; break;
            case eros::Node::State::INITIALIZING: color = eros_window::Color::YELLOW_COLOR; break;
            case eros::Node::State::INITIALIZED: color = eros_window::Color::YELLOW_COLOR; break;
            case eros::Node::State::RUNNING: color = eros_window::Color::BLUE_COLOR; break;
            case eros::Node::State::PAUSED: color = eros_window::Color::GREEN_COLOR; break;
            case eros::Node::State::RESET: color = eros_window::Color::YELLOW_COLOR; break;
            case eros::Node::State::FINISHED: color = eros_window::Color::YELLOW_COLOR; break;
            default: color = eros_window::Color::RED_COLOR; break;
        }

        wattron(get_window(), COLOR_PAIR(color));
        std::string str = get_node_info((*node_it), index == get_selected_record());
        mvwprintw(
            get_window(), TASKSTART_COORD_Y + 2 + (int)index, TASKSTART_COORD_X + 1, str.c_str());
        wclrtoeol(get_window());
        wattroff(get_window(), COLOR_PAIR(color));
        index++;
    }
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
bool NodeWindow::insertNode(NodeType node_type,
                            std::string device,
                            std::string base_node_name,
                            std::string node_name) {
    std::lock_guard<std::mutex> guard(node_list_mutex);
    std::size_t before = node_list.size();
    NodeData newNode(node_list.size(), node_type, device, base_node_name, node_name);
    newNode.state = eros::Node::State::RUNNING;
    node_list.push_back(newNode);
    std::size_t after = node_list.size();
    update_record_count((uint16_t)after);
    return after > before;
}
eros_window::KeyEventContainer NodeWindow::new_keyevent(int key) {
    eros_window::KeyEventContainer output;
    eros_window::MessageText message;
    if (std::find(supported_keys.begin(), supported_keys.end(), key) != supported_keys.end()) {
        output.message.level =
            eros::Level::Type::ERROR;  // Set default Level to error, so if any supported keys
                                       // are not processed, will actively fail.
    }
    else {
        return output;
    }
    logger->log_debug("Key: " + std::to_string(key));
    if (focused == true) {
        if (key == KEY_UP) {
            decrement_selected_record();
        }
        else if (key == KEY_DOWN) {
            increment_selected_record();
        }
        else if ((key == eros_window::KEY_f) || (key == eros_window::KEY_F)) {
            if (node_list.size() == 0) {
                return output;
            }
            auto node = node_list.at((uint16_t)get_selected_record());
            if (node.type == NodeType::EROS) {
                std::string firmware_topic = "/" + node.node_name + "/srv_firmware";
                logger->log_warn("Client Topic: " + firmware_topic);
                if (nodeHandle == nullptr) {
                    logger->log_error("Node Handle has no memory!");
                }
                ros::ServiceClient client =
                    nodeHandle->serviceClient<eros::srv_firmware>(firmware_topic);
                eros::srv_firmware srv;
                if (client.call(srv)) {
                    message = eros_window::MessageText(
                        "Firmware: Node: " + srv.response.NodeName +
                            " Version: " + std::to_string(srv.response.MajorRelease) + "." +
                            std::to_string(srv.response.MinorRelease) + "." +
                            std::to_string(srv.response.BuildNumber) +
                            " Desc: " + srv.response.Description,
                        eros::Level::Type::INFO);
                }
                else {
                    std::string str = "Node: " + node.node_name + " Firmware Check Failed!";
                    message = eros_window::MessageText(str, eros::Level::Type::WARN);
                    logger->log_warn(str);
                }
            }
            else {
                std::string str = "Node: " + node.node_name + " is not an EROS Node.";
                message = eros_window::MessageText(str, eros::Level::Type::WARN);
                logger->log_warn(str);
            }
        }
        else if ((key == eros_window::KEY_l) || (key == eros_window::KEY_L)) {
            std::string str = "Enter new Log Level ";
            for (uint8_t i = (uint8_t)eros::Level::Type::UNKNOWN;
                 i < (uint8_t)eros::Level::Type::END_OF_LIST;
                 ++i) {
                if (i == (uint8_t)eros::Level::Type::UNKNOWN) {
                    // Do nothing
                }
                else {
                    str += std::to_string(i) + ":" +
                           eros::Level::LevelString((eros::Level::Type)i) + " ";
                }
            }
            message = eros_window::MessageText(str, eros::Level::Type::INFO);
        }
        else if ((key == eros_window::KEY_n) || (key == eros_window::KEY_N)) {
            std::string str = "Enter new Node State ";
            for (uint8_t i = (uint8_t)eros::Node::State::UNKNOWN;
                 i < (uint8_t)eros::Node::State::END_OF_LIST;
                 ++i) {
                if (i == (uint8_t)eros::Node::State::UNKNOWN) {
                    // Do nothing
                }
                else {
                    str += std::to_string(i) + ":" +
                           eros::Node::NodeStateString((eros::Node::State)i) + " ";
                }
            }
            message = eros_window::MessageText(str, eros::Level::Type::INFO);
        }
        else if ((key == eros_window::KEY_d) || (key == eros_window::KEY_D)) {
            if (node_list.size() == 0) {
                return output;
            }
            auto node = node_list.at((uint16_t)get_selected_record());
            output.command.type = eros_window::WindowCommandType::VIEW_DIAGNOSTICS_NODE;
            output.command.option = node.node_name;
            std::string str = "Requesting Diagnostics for Node: " + node.node_name;
            message = eros_window::MessageText(str, eros::Level::Type::INFO);
            logger->log_debug(str);
        }
        else if ((key == eros_window::KEY_1) || (key == eros_window::KEY_2) ||
                 (key == eros_window::KEY_3) || (key == eros_window::KEY_4) ||
                 (key == eros_window::KEY_5) || (key == eros_window::KEY_6) ||
                 (key == eros_window::KEY_7) || (key == eros_window::KEY_8) ||
                 (key == eros_window::KEY_9)) {
            if (node_list.size() == 0) {
                return output;
            }
            if ((previous_key == eros_window::KEY_l) || (previous_key == eros_window::KEY_L)) {
                uint16_t verbosity_value = key - (eros_window::KEY_1) + 1;
                std::string verbosity_level =
                    eros::Level::LevelString((eros::Level::Type)verbosity_value);
                if (verbosity_level == "UNKNOWN") {
                    std::string str = "Requested Log Level Not Supported.";
                    message = eros_window::MessageText(str, eros::Level::Type::WARN);
                    logger->log_warn(str);
                }
                auto node = node_list.at((uint16_t)get_selected_record());
                std::string logger_level_topic = "/" + node.node_name + "/srv_loggerlevel";
                ros::ServiceClient client =
                    nodeHandle->serviceClient<eros::srv_logger_level>(logger_level_topic);
                eros::srv_logger_level srv;
                srv.request.LoggerLevel = verbosity_level;
                if (client.call(srv)) {
                    std::string str = "Node: " + node.node_name + " " + srv.response.Response;
                    message = eros_window::MessageText(str, eros::Level::Type::INFO);
                }
                else {
                    std::string str = "Node: " + node.node_name + " Logger Level Change Failed!";
                    message = eros_window::MessageText(str, eros::Level::Type::WARN);
                    logger->log_warn(str);
                }
            }
            if ((previous_key == eros_window::KEY_n) || (previous_key == eros_window::KEY_N)) {
                uint16_t state_value = key - (eros_window::KEY_1) + 1;
                std::string req_state = eros::Node::NodeStateString((eros::Node::State)state_value);
                if (req_state == "UNKNOWN") {
                    std::string str = "Requested Node State Not Supported.";
                    message = eros_window::MessageText(str, eros::Level::Type::WARN);
                    logger->log_warn(str);
                }

                auto node = node_list.at((uint16_t)get_selected_record());
                std::string nodestate_topic = "/" + node.node_name + "/srv_nodestate_change";
                ros::ServiceClient client =
                    nodeHandle->serviceClient<eros::srv_change_nodestate>(nodestate_topic);
                eros::srv_change_nodestate srv;
                srv.request.RequestedNodeState = req_state;
                if (client.call(srv)) {
                    if (req_state == srv.response.NodeState) {
                        std::string str = "Node: " + node.node_name +
                                          " New Node State: " + srv.response.NodeState;
                        message = eros_window::MessageText(str, eros::Level::Type::INFO);
                    }
                    else {
                        std::string str = "Node: " + node.node_name +
                                          " Requested State: " + req_state + " Rejected! ";
                        message = eros_window::MessageText(str, eros::Level::Type::WARN);
                        logger->log_warn(str);
                    }
                }
                else {
                    std::string str =
                        "Node: " + node.node_name + " Node State Change Request Failed! ";
                    message = eros_window::MessageText(str, eros::Level::Type::WARN);
                    logger->log_warn(str);
                }
            }
        }
        else {
        }
    }
    previous_key = key;
    output.message = message;
    return output;
}

bool NodeWindow::new_msg(eros::heartbeat heartbeat_msg) {
    node_list_mutex.lock();
    for (std::vector<NodeData>::iterator node_it = node_list.begin(); node_it != node_list.end();
         ++node_it) {
        if (node_it->node_name == heartbeat_msg.NodeName) {
            node_it->last_heartbeat_delta = 0.0;
            node_it->last_heartbeat = t_ros_time_;
            node_it->host_device = heartbeat_msg.HostName;
            node_it->base_node_name = heartbeat_msg.BaseNodeName;
            node_it->state = (eros::Node::State)heartbeat_msg.NodeState;
            node_list_mutex.unlock();
            return true;
        }
    }
    node_list_mutex.unlock();
    return insertNode(
        NodeType::EROS, "Unknown", heartbeat_msg.BaseNodeName, heartbeat_msg.NodeName);
}
std::vector<std::string> NodeWindow::get_node_info() {
    std::vector<std::string> strs;
    for (std::vector<NodeData>::iterator node_it = node_list.begin(); node_it != node_list.end();
         ++node_it) {
        std::string str = get_node_info((*node_it), false);
        strs.push_back(str);
    }
    return strs;
}
bool NodeWindow::new_msg(eros::resource resource_used_msg) {
    node_list_mutex.lock();
    for (std::vector<NodeData>::iterator node_it = node_list.begin(); node_it != node_list.end();
         ++node_it) {
        if (node_it->node_name == resource_used_msg.Name) {
            node_it->last_heartbeat_delta = 0.0;
            node_it->last_heartbeat = t_ros_time_;
            node_it->pid = resource_used_msg.PID;  // todo: Handle PID # change
            node_it->cpu_used_perc = resource_used_msg.CPU_Perc;
            node_it->mem_used_perc = resource_used_msg.RAM_Perc;
            node_list_mutex.unlock();
            return true;
        }
    }
    node_list_mutex.unlock();
    return insertNode(NodeType::EROS, "Unknown", "Unknown", resource_used_msg.Name);
}
std::string NodeWindow::get_node_info(NodeData node, bool selected) {
    std::string str = "";
    std::size_t width = 0;
    {
        width = node_window_fields.find(NodeFieldColumn::MARKER)->second.width;
        for (std::size_t i = 0; i < width; ++i) {
            if (selected == true) {
                str += "*";
            }
            else {
                str += " ";
            }
        }
    }
    {
        width = node_window_fields.find(NodeFieldColumn::ID)->second.width;
        std::string tempstr = std::to_string(node.id);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::HOSTNAME)->second.width;
        std::string tempstr = node.host_device;
        if (tempstr.size() > width) {
            tempstr = tempstr.substr(0, width - 4) + "... ";
        }
        else {
            if (tempstr.size() > (std::size_t)(width - 1)) {
                tempstr = tempstr.substr(0, (width - 1));
                tempstr += " ";
            }
            else {
                std::size_t spaces = width - tempstr.size();
                if (spaces > 0) {
                    tempstr += std::string(spaces, ' ');
                }
            }
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::NODENAME)->second.width;
        std::string tempstr = node.node_name;
        std::size_t found_hostname = node.node_name.find(node.host_device);
        if (found_hostname != std::string::npos) {
            tempstr.replace(found_hostname, node.host_device.length(), "");
        }
        if (tempstr.size() > width) {
            tempstr = tempstr.substr(0, width - 4) + "... ";
        }
        else {
            std::size_t spaces = width - tempstr.size();
            if (spaces > 0) {
                tempstr += std::string(spaces, ' ');
            }
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::STATUS)->second.width;
        std::string tempstr = eros::Node::NodeStateString(node.state);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::RESTARTS)->second.width;
        std::string tempstr = std::to_string(node.restart_count);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::PID)->second.width;
        std::string tempstr = std::to_string(node.pid);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::CPU)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", node.cpu_used_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::RAM)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", node.mem_used_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::RX)->second.width;
        std::string max_number_str(width - 4, '9');
        double max_num = std::atof(max_number_str.c_str()) + 0.99;
        if (node.last_heartbeat_delta > max_num) {
            node.last_heartbeat_delta = max_num;
        }
        char tempstr[2 * width];
        sprintf(tempstr, "%2.2f", node.last_heartbeat_delta);
        std::string tempstr_str = std::string(tempstr);
        std::size_t spaces = width - tempstr_str.size();
        if (spaces > 0) {
            tempstr_str += std::string(spaces, ' ');
        }
        str += tempstr_str;
    }

    return str;
}
}  // namespace eros_nodes::SystemMonitor