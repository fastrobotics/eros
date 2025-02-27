#include "DeviceWindow/DeviceWindow.h"
namespace eros_nodes::SystemMonitor {
constexpr double DeviceWindow::START_X_PERC;
constexpr double DeviceWindow::START_Y_PERC;
constexpr double DeviceWindow::WIDTH_PERC;
constexpr double DeviceWindow::HEIGHT_PERC;
DeviceWindow::~DeviceWindow() {
}
bool DeviceWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    std::map<std::string, DeviceData>::iterator device_it = device_list.begin();
    while (device_it != device_list.end()) {
        device_it->second.last_heartbeat_delta += dt;
        if (device_it->second.last_heartbeat_delta >= 4.0 * eros_window::COMMTIMEOUT_THRESHOLD) {
            device_it->second.state = eros::Node::State::UNKNOWN;
        }
        ++device_it;
    }
    status = update_window();
    return status;
}
bool DeviceWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    // GCOVR_EXCL_START
    std::string header = get_deviceheader();
    mvwprintw(get_window(), 1, 1, header.c_str());
    std::string dashed(get_screen_coordinates_pixel().width_pix - 2, '-');
    mvwprintw(get_window(), 2, 1, dashed.c_str());
    const uint16_t DEVICESTART_COORD_Y = 1;
    const uint16_t DEVICESTART_COORD_X = 1;
    uint16_t index = 0;
    std::map<std::string, DeviceData>::iterator device_it;
    for (device_it = device_list.begin(); device_it != device_list.end(); device_it++) {
        eros_window::Color color = eros_window::Color::UNKNOWN;
        switch (device_it->second.state) {
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
        std::string str = get_device_info(device_it->second, false);
        wattron(get_window(), COLOR_PAIR(color));
        mvwprintw(get_window(),
                  DEVICESTART_COORD_Y + 2 + (int)index,
                  DEVICESTART_COORD_X + 1,
                  str.c_str());
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
std::string DeviceWindow::pretty() {
    std::string str = "-----Device Window-----\n";
    if (device_list.size() == 0) {
        str += "\tNo Devices Found.";
    }
    else {
        str += get_deviceheader() + "\n";
        std::map<std::string, DeviceData>::iterator device_it = device_list.begin();
        while (device_it != device_list.end()) {
            str += get_device_info(device_it->second, false) + "\n";
            ++device_it;
        }
    }
    return str;
}
bool DeviceWindow::new_msg(eros::resource resource_available_msg) {
    device_list_mutex.lock();
    if (resource_available_msg.Name == "") {
        return false;
    }
    std::map<std::string, DeviceData>::iterator device_it;

    device_it = device_list.find(resource_available_msg.Name);
    if (device_it != device_list.end()) {  // Found it, update the record
        device_it->second.cpu_av_perc = resource_available_msg.CPU_Perc;
        device_it->second.ram_av_perc = resource_available_msg.RAM_Perc;
        device_it->second.disk_av_perc = resource_available_msg.DISK_Perc;
        device_it->second.state = eros::Node::State::RUNNING;
        device_it->second.last_heartbeat_delta = 0.0;
    }
    else {  // Didn't find it,create one
        device_list_mutex.unlock();
        return insertDevice(resource_available_msg);
    }
    device_list_mutex.unlock();
    return true;
}
bool DeviceWindow::new_msg(eros::loadfactor loadfactor_msg) {
    device_list_mutex.lock();
    std::map<std::string, DeviceData>::iterator device_it;

    device_it = device_list.find(loadfactor_msg.DeviceName);
    if (device_it != device_list.end()) {  // Found it, update the record
        if (loadfactor_msg.loadfactor.size() != 3) {
            return false;
        }
        device_it->second.load_factor.at(0) = loadfactor_msg.loadfactor.at(0);
        device_it->second.load_factor.at(1) = loadfactor_msg.loadfactor.at(1);
        device_it->second.load_factor.at(2) = loadfactor_msg.loadfactor.at(2);
        device_it->second.state = eros::Node::State::RUNNING;
        device_it->second.last_heartbeat_delta = 0.0;
    }
    else {  // Didn't find it,create one
        device_list_mutex.unlock();
        return insertDevice(loadfactor_msg);
    }
    device_list_mutex.unlock();
    return true;
}
bool DeviceWindow::insertDevice(eros::resource resource_data) {
    std::lock_guard<std::mutex> guard(device_list_mutex);
    std::size_t before = device_list.size();
    DeviceData newDevice(device_list.size(), resource_data.Name);
    newDevice.cpu_av_perc = resource_data.CPU_Perc;
    newDevice.ram_av_perc = resource_data.RAM_Perc;
    newDevice.disk_av_perc = resource_data.DISK_Perc;
    std::string key = newDevice.name;
    device_list.insert(std::pair<std::string, DeviceData>(key, newDevice));
    std::size_t after = device_list.size();
    return after > before;
}
bool DeviceWindow::insertDevice(eros::loadfactor loadfactor_data) {
    std::lock_guard<std::mutex> guard(device_list_mutex);
    std::size_t before = device_list.size();
    DeviceData newDevice(device_list.size(), loadfactor_data.DeviceName);
    if (loadfactor_data.loadfactor.size() != 3) {
        return false;
    }
    newDevice.load_factor.at(0) = loadfactor_data.loadfactor.at(0);
    newDevice.load_factor.at(1) = loadfactor_data.loadfactor.at(1);
    newDevice.load_factor.at(2) = loadfactor_data.loadfactor.at(2);
    std::string key = newDevice.name;
    device_list.insert(std::pair<std::string, DeviceData>(key, newDevice));
    std::size_t after = device_list.size();
    return after > before;
}
std::string DeviceWindow::get_device_info(DeviceData device, bool selected) {
    std::string str = "";
    std::size_t width = 0;
    {
        width = device_window_fields.find(DeviceFieldColumn::MARKER)->second.width;
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
        width = device_window_fields.find(DeviceFieldColumn::ID)->second.width;
        std::string tempstr = std::to_string(device.id);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = device_window_fields.find(DeviceFieldColumn::NAME)->second.width;
        std::string tempstr = device.name;
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
        width = device_window_fields.find(DeviceFieldColumn::CPU)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", device.cpu_av_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }

    {
        width = device_window_fields.find(DeviceFieldColumn::RAM)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", device.ram_av_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = device_window_fields.find(DeviceFieldColumn::DISK)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", device.disk_av_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = device_window_fields.find(DeviceFieldColumn::LOADFACTOR)->second.width;
        char c_tempstr[128];
        sprintf(c_tempstr,
                "[%3.2f,%3.2f,%3.2f]",
                device.load_factor.at(0),
                device.load_factor.at(1),
                device.load_factor.at(2));
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = device_window_fields.find(DeviceFieldColumn::RX)->second.width;
        std::string max_number_str(width - 4, '9');
        double max_num = std::atof(max_number_str.c_str()) + 0.99;
        if (device.last_heartbeat_delta > max_num) {
            device.last_heartbeat_delta = max_num;
        }
        char tempstr[2 * width];
        sprintf(tempstr, "%2.2f", device.last_heartbeat_delta);
        std::string tempstr_str = std::string(tempstr);
        std::size_t spaces = width - tempstr_str.size();
        if (spaces > 0) {
            tempstr_str += std::string(spaces, ' ');
        }
        str += tempstr_str;
    }

    return str;
}
std::string DeviceWindow::get_deviceheader() {
    std::string str = "";
    std::map<DeviceFieldColumn, eros_window::Field>::iterator it = device_window_fields.begin();
    while (it != device_window_fields.end()) {
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
        logger->log_warn("Device Header too long for Window!.");
        return "";
    }
    return str;
}
}  // namespace eros_nodes::SystemMonitor