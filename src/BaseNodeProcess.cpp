#include <eros/BaseNodeProcess.h>
using namespace eros;
eros_diagnostic::Diagnostic BaseNodeProcess::base_update(double t_dt, double t_system_time) {
    run_time += t_dt;
    system_time = t_system_time;
    eros_diagnostic::Diagnostic diag =
        diagnostic_manager.update_diagnostic(eros_diagnostic::DiagnosticType::SOFTWARE,
                                             Level::Type::DEBUG,
                                             eros_diagnostic::Message::NOERROR,
                                             "Base Process Updated.");
    if (node_state == Node::State::START) {
        request_statechange(Node::State::INITIALIZING);
    }
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> BaseNodeProcess::base_new_commandmsg(eros::command t_msg) {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    Command::Type cmd_type = (Command::Type)t_msg.Command;
    bool found_it = false;
    for (auto supported_cmd_type : supported_commands) {
        if (supported_cmd_type == cmd_type) {
            found_it = true;
            // Don't do anything, including adding a diagnostic. The concrete class should do that.
        }
    }
    if (found_it == false) {
        eros_diagnostic::Diagnostic diag = diagnostic_manager.update_diagnostic(
            eros_diagnostic::DiagnosticType::SOFTWARE,
            Level::Type::DEBUG,
            eros_diagnostic::Message::NOERROR,
            "Command Type: " + Command::CommandString(cmd_type) + " Not Enabled.  Ignoring.");
        diag_list.push_back(diag);
    }
    return diag_list;
}
bool BaseNodeProcess::request_statechange(Node::State newstate, bool override) {
    Node::State current_state = node_state;
    bool state_changed = false;
    if (override) {
        if (current_state != newstate) {
            state_changed = true;
        }
        current_state = newstate;
    }
    else {
        switch (current_state) {
            case Node::State::START:
                if (newstate == Node::State::INITIALIZING) {
                    state_changed = true;
                }
                else {
                    state_changed = false;
                }
                break;
            case Node::State::INITIALIZING:
                if (newstate == Node::State::INITIALIZED) {
                    state_changed = true;
                }
                else {
                    state_changed = false;
                }
                break;
            case Node::State::INITIALIZED:
                if (newstate == Node::State::RUNNING) {
                    state_changed = true;
                }
                else {
                    state_changed = false;
                }
                break;
            case Node::State::RUNNING:
                if (newstate == Node::State::PAUSED) {
                    state_changed = true;
                }
                else if (newstate == Node::State::RESET) {
                    state_changed = true;
                }
                else if (newstate == Node::State::FINISHED) {
                    state_changed = true;
                }
                else {
                    state_changed = false;
                }
                break;
            case Node::State::PAUSED:
                if (newstate == Node::State::RUNNING) {
                    state_changed = true;
                }
                else {
                    state_changed = false;
                }
                break;
            case Node::State::RESET:
                if (newstate == Node::State::INITIALIZING) {
                    state_changed = true;
                }
                else if (newstate == Node::State::RUNNING) {
                    state_changed = true;
                }
                else {
                    state_changed = false;
                }
                break;
            case Node::State::FINISHED: state_changed = false; break;
            // No practical way to enter, keeping just in case.
            // LCOV_EXCL_START
            default:
                state_changed = false;
                break;
                // LCOV_EXCL_STOP
        }
    }
    if (state_changed == true) {
        node_state = newstate;
    }
    else {
        logger->log_info("State Change from: " + Node::NodeStateString(current_state) +
                         " To State: " + Node::NodeStateString(newstate) + " Rejected.");
    }
    return state_changed;
}

void BaseNodeProcess::base_cleanup() {
    return;
}
json BaseNodeProcess::read_configuration(std::string device_name,
                                         bool include_self,
                                         std::string file_path) {
    json j_obj;
    json empty;
    if (file_path.size() == 0) {
        logger->log_error("No file path defined.");
        return empty;
    }
    file_path = sanitize_path(file_path);
    std::ifstream fd(file_path);
    if (fd.is_open() == false) {
        logger->log_error("Unable to read file.");
        return empty;
    }
    json j;
    fd >> j_obj;
    if (include_self == true) {
        for (auto it = j_obj["DeviceList"].begin(); it != j_obj["DeviceList"].end(); ++it) {
            if (it.key() == device_name) {
                j[it.key()] = it.value();
            }
        }
    }
    for (auto it = j_obj["DeviceList"].begin(); it != j_obj["DeviceList"].end(); ++it) {
        if (it.value()["Parent"] == device_name) {
            j[it.key()] = it.value();
        }
    }
    return j;
}
std::string BaseNodeProcess::sanitize_path(std::string path) {
    if (path.at(0) == '~') {
        path.erase(path.begin() + 0);
        path = std::string(getenv("HOME")) + path;
    }
    return path;
}

FileHelper::FileInfo BaseNodeProcess::read_file(std::string file_path) {
    FileHelper::FileInfo fileInfo;
    file_path = sanitize_path(file_path);
    file_path = boost::filesystem::absolute(file_path).string();
    fileInfo.full_path = file_path;
    std::string extension = boost::filesystem::extension(file_path);
    if (extension == ".zip") {
        fileInfo.fileType = FileHelper::FileType::ZIP;
    }
    else {
        fileInfo.fileType = FileHelper::FileType::UNKNOWN;
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
        return fileInfo;
    }
    auto p = boost::filesystem::path(file_path);
    fileInfo.folder = p.parent_path().string() + "/";
    fileInfo.file_name = p.filename().string();
    std::ifstream fl(file_path);
    if (fl.is_open() == false) {
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
        return fileInfo;
    }
    fl.seekg(0, std::ios::end);
    std::size_t len = fl.tellg();
    char *ret = new char[len];
    fl.seekg(0, std::ios::beg);
    fl.read(ret, len);
    fl.close();
    fileInfo.data = ret;
    fileInfo.byte_size = (uint64_t)len;
    if (fileInfo.byte_size > 0) {
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_OK;
    }
    else {
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
    }
    return fileInfo;
}
FileHelper::FileInfo BaseNodeProcess::write_file(std::string full_path,
                                                 char *bytes,
                                                 uint64_t byte_count) {
    FileHelper::FileInfo fileInfo;
    full_path = sanitize_path(full_path);
    full_path = boost::filesystem::absolute(full_path).string();
    fileInfo.full_path = full_path;
    std::string extension = boost::filesystem::extension(full_path);
    if (extension == ".zip") {
        fileInfo.fileType = FileHelper::FileType::ZIP;
    }
    else {
        fileInfo.fileType = FileHelper::FileType::UNKNOWN;
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
        return fileInfo;
    }
    auto p = boost::filesystem::path(full_path);
    fileInfo.folder = p.parent_path().string() + "/";
    fileInfo.file_name = p.filename().string();
    std::ofstream file(full_path.c_str(), std::ios::binary);
    if (file.is_open() == false) {
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
        return fileInfo;
    }
    file.write(bytes, byte_count);
    file.close();
    fileInfo.data = bytes;
    fileInfo.byte_size = byte_count;
    fileInfo.fileStatus = FileHelper::FileStatus::FILE_OK;
    return fileInfo;
}
std::vector<std::string> BaseNodeProcess::get_files_indir(std::string dir) {
    std::vector<std::string> files;
    std::string ls_cmd = "ls " + dir;
    ExecResult execResult = eros_utility::CoreUtility::exec(ls_cmd.c_str(), true);
    std::string res = execResult.Result;
    boost::split(files, res, boost::is_any_of("\n"), boost::token_compress_on);
    for (std::vector<std::string>::iterator it = files.begin(); it != files.end();) {
        if (it->size() == 0) {
            it = files.erase(it);
        }
        else {
            ++it;
        }
    }
    return files;
}