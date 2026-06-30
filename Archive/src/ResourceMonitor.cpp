#include <eros/ResourceMonitor.h>

namespace eros {
ResourceMonitor::ResourceMonitor(std::string name,
                                 Mode _mode,
                                 eros_diagnostic::Diagnostic _diag,
                                 Logger *_logger)
    : mode(_mode),
      architecture(Architecture::Type::UNKNOWN),
      diagnostic(_diag),
      logger(_logger),
      initialized(false),
      run_time(0.0),
      processor_count(0) {
    resourceInfo.process_name = name;
    resourceInfo.cpu_perc = -1.0;
    resourceInfo.disk_perc = -1.0;
    resourceInfo.ram_perc = -1.0;
    diagnostic.type = eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE;
    load_factor.DeviceName = name;
    load_factor.loadfactor.push_back(0.0);
    load_factor.loadfactor.push_back(0.0);
    load_factor.loadfactor.push_back(0.0);
}
ResourceMonitor::~ResourceMonitor() {
}
eros_diagnostic::Diagnostic ResourceMonitor::init() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    ExecResult execResult;
    architecture = read_device_architecture();
    // The following can't currently be checked for code coverage as it depends on the device being
    // run on. LCOV_EXCL_START
    if (architecture == Architecture::Type::UNKNOWN) {
        diag.level = Level::Type::ERROR;
        diag.message = eros_diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Architecture Not Supported.";
        diag.update_count++;
        logger->log_diagnostic(diag);
        initialized = false;
    }
    // LCOV_EXCL_STOP

    // The following can't currently be checked for code coverage as it depends on the device being
    // run on. LCOV_EXCL_START
    if ((architecture == Architecture::Type::UNKNOWN)) {
        diag.level = Level::Type::ERROR;
        diag.message = eros_diagnostic::Message::INITIALIZING_ERROR;
        diag.description =
            "Architecture: " + Architecture::ArchitectureString(architecture) + " Not Supported.";
        logger->log_diagnostic(diag);
        diag.update_count++;
        initialized = false;
    }
    // LCOV_EXCL_STOP
    resourceInfo.pid = ::getpid();
    try {
        execResult = eros_utility::CoreUtility::exec("nproc", true);
        processor_count = std::atoi(execResult.Result.c_str());
    }
    // The following can't currently be checked for code coverage as it depends on the device being
    // run on.
    // LCOV_EXCL_START
    catch (const std::exception &e) {
        std::string tempstr = "Unable to determine number of processors: " + std::string(e.what());

        diag.level = Level::Type::ERROR;
        diag.message = eros_diagnostic::Message::INITIALIZING_ERROR;
        diag.description = tempstr;
        diag.update_count++;
        logger->log_error(tempstr);
        logger->log_diagnostic(diag);
        initialized = false;
    }
    // LCOV_EXCL_STOP
    if (mode == Mode::PROCESS) {
        diag = read_process_resource_usage();
    }
    else if (mode == Mode::DEVICE) {
        diag = read_device_resource_availability();
        // The following can't currently be checked for code coverage as it depends on the device
        // being run on.
        // LCOV_EXCL_START
        if (diag.level > Level::Type::WARN) {
            logger->log_diagnostic(diag);
            return diag;
        }
        // LCOV_EXCL_STOP
        diag = read_device_loadfactor();
    }
    if (diag.level <= Level::Type::NOTICE) {
        initialized = true;
        logger->log_diagnostic(diag);
    }
    // The following can't currently be checked for code coverage as it depends on the device being
    // run on.
    // LCOV_EXCL_START
    else {
        initialized = false;
    }
    // LCOV_EXCL_STOP
    logger->log_diagnostic(diag);
    return diag;
}
bool ResourceMonitor::reset() {
    ResourceInfo reset_info;
    reset_info.process_name = resourceInfo.process_name;
    reset_info.pid = resourceInfo.pid;
    reset_info.cpu_perc = 0.0;
    reset_info.ram_perc = 0.0;
    reset_info.disk_perc = 0.0;
    resourceInfo = reset_info;
    return true;
}
eros_diagnostic::Diagnostic ResourceMonitor::update(double t_dt) {
    eros_diagnostic::Diagnostic diag = diagnostic;
    if (initialized == false) {
        diag.level = Level::Type::ERROR;
        diag.message = eros_diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Architecture Not Supported.";
        diag.update_count++;
        diagnostic = diag;
        return diag;
    }
    run_time += t_dt;
    if (mode == Mode::PROCESS) {
        diag = read_process_resource_usage();
    }
    else if (mode == Mode::DEVICE) {
        diag = read_device_resource_availability();
        // The following can't currently be checked for code coverage as it depends on the device
        // being run on.
        // LCOV_EXCL_START
        if (diag.level > Level::Type::WARN) {
            return diag;
        }
        // LCOV_EXCL_STOP
        diag = read_device_loadfactor();
    }
    diagnostic = diag;
    return diag;
}
eros_diagnostic::Diagnostic ResourceMonitor::read_process_resource_usage() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string top_query =
        "top -b -n 2 -d 0.2 -p " + std::to_string(resourceInfo.pid) + " | tail -1";
    ExecResult execResult;
    execResult = eros_utility::CoreUtility::exec(top_query.c_str(), true);
    std::string res = execResult.Result;
    std::vector<std::string> strs;
    boost::algorithm::split(strs, res, boost::is_any_of("\t "), boost::token_compress_on);
    resourceInfo.stamp = ros::Time::now();
    if (strs.at(0) == "") {
        strs.erase(strs.begin());
    }
    if (architecture == Architecture::Type::X86_64) {
        // The following can't currently be checked for code coverage as it depends on the device
        // being run on.
        // LCOV_EXCL_START
        if (strs.size() != 12) {
            for (std::size_t i = 0; i < strs.size(); ++i) {
                printf("[%d/%d] %s\n", (int)i, (int)strs.size(), strs.at(i).c_str());
            }
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.description = "Improper string to process (size != 12): " + res;
            diag.update_count++;
            return diag;
        }
        // LCOV_EXCL_STOP
        try {
            resourceInfo.cpu_perc = std::atof(strs.at(8).c_str());
            resourceInfo.ram_perc = std::atof(strs.at(9).c_str());
        }
        // The following can't currently be checked for code coverage as it depends on the device
        // being run on.
        // LCOV_EXCL_START
        catch (const std::exception &e) {
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.description =
                "Unable to process string: " + res + " with result: " + std::string(e.what());
            diag.update_count++;
            return diag;
        }
        // LCOV_EXCL_STOP
    }
    // The following can't currently be checked for code coverage as it depends on the device being
    // run on.
    // LCOV_EXCL_START
    else if (architecture == Architecture::Type::AARCH64) {
        if (strs.size() != 12) {
            for (std::size_t i = 0; i < strs.size(); ++i) {
                printf("[%d/%d] %s\n", (int)i, (int)strs.size(), strs.at(i).c_str());
            }
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.description = "Improper string to process: (size != 12)" + res;
            diag.update_count++;
            return diag;
        }
        try {
            resourceInfo.cpu_perc = std::atof(strs.at(8).c_str());
            resourceInfo.ram_perc = std::atof(strs.at(9).c_str());
        }
        catch (const std::exception &e) {
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.description =
                "Unable to process string: " + res + " with result: " + std::string(e.what());
            diag.update_count++;
            return diag;
        }
    }
    // LCOV_EXCL_STOP
    // The following can't currently be checked for code coverage as it depends on the device being
    // run on.
    // LCOV_EXCL_START
    else if (architecture == Architecture::Type::ARMV7L) {
        if (strs.size() != 12) {
            for (std::size_t i = 0; i < strs.size(); ++i) {
                printf("[%d/%d] %s\n", (int)i, (int)strs.size(), strs.at(i).c_str());
            }
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.description = "Improper string to process: (size != 13)" + res;
            diag.update_count++;
            return diag;
        }
        try {
            resourceInfo.cpu_perc = std::atof(strs.at(8).c_str());
            resourceInfo.ram_perc = std::atof(strs.at(9).c_str());
        }
        catch (const std::exception &e) {
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.description =
                "Unable to process string: " + res + " with result: " + std::string(e.what());
            diag.update_count++;
            return diag;
        }
    }
    // LCOV_EXCL_STOP
    // The following can't currently be checked for code coverage as it depends on the device being
    // run on.
    // LCOV_EXCL_START
    else {
        diag.level = Level::Type::ERROR;
        diag.message = eros_diagnostic::Message::INITIALIZING_ERROR;
        diag.description =
            "Architecture: " + Architecture::ArchitectureString(architecture) + " Not Supported.";
        diag.update_count++;
        return diag;
    }
    // LCOV_EXCL_STOP
    diag.level = Level::Type::INFO;
    diag.message = eros_diagnostic::Message::NOERROR;
    diag.description = "Updated.";
    diag.update_count++;
    return diag;
}
eros_diagnostic::Diagnostic ResourceMonitor::read_device_resource_availability() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    {  // Read Free CPU
        std::string res;
        if (architecture != Architecture::Type::UNKNOWN) {
            try {
                std::string top_query = "top -bn1 | grep '%Cpu(s)'";
                ExecResult execResult;
                execResult = eros_utility::CoreUtility::exec(top_query.c_str(), true);
                std::string res = execResult.Result;
                std::vector<std::string> strs;
                boost::algorithm::split(
                    strs, res, boost::is_any_of("\t "), boost::token_compress_on);
                bool found_me = false;
                for (std::size_t i = 0; i < strs.size(); ++i) {
                    if (strs.at(i).find("id") != std::string::npos) {
                        if (i > 0) {
                            found_me = true;
                            resourceInfo.cpu_perc = std::atof(strs.at(i - 1).c_str());
                        }
                    }
                }
                // The following can't currently be checked for code coverage as it depends on the
                // device being run on.
                // LCOV_EXCL_START
                if (found_me == false) {
                    diag.level = Level::Type::ERROR;
                    diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
                    diag.description = "Unable to process string: " + res;
                    diag.update_count++;
                    return diag;
                }
                // LCOV_EXCL_STOP
            }
            // The following can't currently be checked for code coverage as it depends on the
            // device being run on.
            // LCOV_EXCL_START
            catch (const std::exception &e) {
                diag.level = Level::Type::ERROR;
                diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
                diag.description =
                    "Unable to process string: " + res + " with result: " + std::string(e.what());
                diag.update_count++;
                return diag;
            }
            // LCOV_EXCL_STOP
        }
        diag.level = Level::Type::INFO;
        diag.message = eros_diagnostic::Message::NOERROR;
        diag.description = "Updated.";
        diag.update_count++;
    }
    {  // Read Free RAM
        std::string res;
        if (architecture != Architecture::Type::UNKNOWN) {
            try {
                std::string top_query = "top -bn1 | grep 'Mem'";
                ExecResult execResult;
                execResult = eros_utility::CoreUtility::exec(top_query.c_str(), true);
                std::string res = execResult.Result;
                std::vector<std::string> strs;
                boost::algorithm::split(
                    strs, res, boost::is_any_of("\t "), boost::token_compress_on);
                int found_count = 0;
                int64_t total_mem = 0;
                int64_t used_mem = 0;
                for (std::size_t i = 0; i < strs.size(); ++i) {
                    if (strs.at(i).find("used") != std::string::npos) {
                        if (i > 0) {
                            found_count++;
                            used_mem = std::atoi(strs.at(i - 1).c_str());
                        }
                    }
                    if (strs.at(i).find("total") != std::string::npos) {
                        if (i > 0) {
                            found_count++;
                            total_mem = std::atoi(strs.at(i - 1).c_str());
                        }
                    }
                    if (found_count == 2) {
                        break;
                    }
                }
                // The following can't currently be checked for code coverage as it depends on the
                // device being run on.
                // LCOV_EXCL_START
                if (found_count != 2) {
                    diag.level = Level::Type::ERROR;
                    diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
                    diag.description = "Unable to process string: " + res;
                    diag.update_count++;
                    return diag;
                }
                // LCOV_EXCL_STOP
                else {
                    resourceInfo.ram_perc =
                        100.0 - (100.0 * (double)(used_mem) / (double)(total_mem));
                }
            }
            // The following can't currently be checked for code coverage as it depends on the
            // device being run on.
            // LCOV_EXCL_START
            catch (const std::exception &e) {
                diag.level = Level::Type::ERROR;
                diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
                diag.description =
                    "Unable to process string: " + res + " with result: " + std::string(e.what());
                diag.update_count++;
                return diag;
            }
            // LCOV_EXCL_STOP
        }
    }
    {  // Read Free Disk Space
        std::string res;
        if (architecture != Architecture::Type::UNKNOWN) {
            try {
                std::string df_query = "df -h";
                ExecResult execResult;
                execResult = eros_utility::CoreUtility::exec(df_query.c_str(), true);
                std::string res = execResult.Result;
                std::vector<std::string> lines;
                boost::split(lines, res, boost::is_any_of("\n"), boost::token_compress_on);
                bool found_me = false;
                for (std::size_t i = 0; i < lines.size(); ++i) {
                    std::vector<std::string> fields;
                    boost::split(
                        fields, lines.at(i), boost::is_any_of(" "), boost::token_compress_on);
                    std::string mount_point = fields.at(fields.size() - 1);
                    if (mount_point == "/") {
                        try {
                            found_me = true;
                            std::string tempstr1 = fields.at(4);

                            tempstr1 =
                                tempstr1.substr(0, tempstr1.size() - 1);  // Remove trailing % sign
                            resourceInfo.disk_perc = 100.0 - std::atof(tempstr1.c_str());
                        }
                        // The following can't currently be checked for code coverage as it depends
                        // on the device being run on.
                        // LCOV_EXCL_START
                        catch (const std::exception &e) {
                            diag.level = Level::Type::ERROR;
                            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
                            diag.description = "Unable to process string: " + res +
                                               " with result: " + std::string(e.what());
                            diag.update_count++;
                            return diag;
                        }
                        // LCOV_EXCL_STOP
                    }
                }
                // The following can't currently be checked for code coverage as it depends on the
                // device being run on.
                // LCOV_EXCL_START
                if (found_me == false) {
                    diag.level = Level::Type::ERROR;
                    diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
                    diag.description = "Unable to process string: " + res;
                    diag.update_count++;
                    return diag;
                }
                // LCOV_EXCL_STOP
            }
            // The following can't currently be checked for code coverage as it depends on the
            // device being run on.
            // LCOV_EXCL_START
            catch (const std::exception &e) {
                diag.level = Level::Type::ERROR;
                diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
                diag.description =
                    "Unable to process string: " + res + " with result: " + std::string(e.what());
                diag.update_count++;
                return diag;
            }
            // LCOV_EXCL_STOP
        }
    }
    diag.level = Level::Type::INFO;
    diag.message = eros_diagnostic::Message::NOERROR;
    diag.description = "Updated.";
    diag.update_count++;
    return diag;
}
eros_diagnostic::Diagnostic ResourceMonitor::read_device_loadfactor() {
    eros_diagnostic::Diagnostic diag = diagnostic;
    std::string res;
    if (architecture != Architecture::Type::UNKNOWN) {
        try {
            std::vector<double> load_factor_data;
            load_factor_data.resize(3);
            std::string top_query = "top -bn1 | grep 'load average:'";
            ExecResult execResult;
            execResult = eros_utility::CoreUtility::exec(top_query.c_str(), true);
            std::string res = execResult.Result;
            std::vector<std::string> strs;
            boost::algorithm::split(strs, res, boost::is_any_of(", "), boost::token_compress_on);
            bool found_me = false;
            for (std::size_t i = 0; i < strs.size(); ++i) {
                if (strs.at(i).find("average") != std::string::npos) {
                    if ((i + 3) <= strs.size()) {
                        load_factor_data.at(0) =
                            std::atof(strs.at(i + 1).c_str()) / (double)(processor_count);
                        load_factor_data.at(1) =
                            std::atof(strs.at(i + 2).c_str()) / (double)processor_count;
                        load_factor_data.at(2) =
                            std::atof(strs.at(i + 3).c_str()) / (double)processor_count;
                        found_me = true;
                        break;
                    }
                }
            }

            // The following can't currently be checked for code coverage as it depends on the
            // device being run on.
            // LCOV_EXCL_START
            if (found_me == false) {
                diag.level = Level::Type::ERROR;
                diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
                diag.description = "Unable to process string: " + res;
                diag.update_count++;
                return diag;
            }
            else {
                load_factor.loadfactor[0] = load_factor_data[0];
                load_factor.loadfactor[1] = load_factor_data[1];
                load_factor.loadfactor[2] = load_factor_data[2];
                load_factor.stamp = ros::Time::now();
            }
            // LCOV_EXCL_STOP
        }
        // The following can't currently be checked for code coverage as it depends on the device
        // being run on.
        // LCOV_EXCL_START
        catch (const std::exception &e) {
            diag.level = Level::Type::ERROR;
            diag.message = eros_diagnostic::Message::DROPPING_PACKETS;
            diag.description =
                "Unable to process string: " + res + " with result: " + std::string(e.what());
            diag.update_count++;
            return diag;
        }
        // LCOV_EXCL_STOP
    }
    diag.level = Level::Type::INFO;
    diag.message = eros_diagnostic::Message::NOERROR;
    diag.description = "Updated.";
    diag.update_count++;
    return diag;
}
Architecture::Type ResourceMonitor::read_device_architecture() {
    // Try 1: Read command: "uname -m"
    {
        std::string cmd = "uname -m";
        ExecResult execResult;
        execResult = eros_utility::CoreUtility::exec(cmd.c_str(), true);
        std::string result = execResult.Result;
        std::size_t found_x86_64 = result.find("x86_64");
        std::size_t found_armv7l = result.find("armv7l");
        std::size_t found_aarch64 = result.find("aarch64");
        if (found_x86_64 != std::string::npos) {
            return Architecture::Type::X86_64;
        }
        // The following can't currently be checked for code coverage as it depends on the device
        // being run on.
        // LCOV_EXCL_START
        else if (found_armv7l != std::string::npos) {
            return Architecture::Type::ARMV7L;
        }
        else if (found_aarch64 != std::string::npos) {
            return Architecture::Type::AARCH64;
        }
        else {
            logger->log_warn("Unexpected result of command: " + cmd + " result: " + result);
        }
        // LCOV_EXCL_STOP
    }
    // The following can't currently be checked for code coverage as it depends on the device
    // being run on.
    // LCOV_EXCL_START
    return Architecture::Type::UNKNOWN;
    // LCOV_EXCL_STOP
}
std::string ResourceMonitor::pretty(ResourceInfo info) {
    std::string str = "--- Resource Monitor Info ---\n";
    str += "\tArchitecture: " + Architecture::ArchitectureString(architecture) +
           " Processor Count: " + std::to_string(processor_count) + "\n";
    str += "\tPID: " + std::to_string(info.pid) + "\n";
    str += "\tRAM: " + std::to_string(info.ram_perc) + "%\n";
    str += "\tCPU: " + std::to_string(info.cpu_perc) + "%\n";
    str += "\tDisk: " + std::to_string(info.disk_perc) + "%\n";

    return str;
}
}  // namespace eros