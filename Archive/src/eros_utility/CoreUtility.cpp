#include <eros_utility/CoreUtility.h>
namespace eros::eros_utility {
bool CoreUtility::isEqual(double a, double b, double eps) {
    double dv = a - b;
    if (fabs(dv) < eps) {
        return true;
    }
    else {
        return false;
    }
}

eros::ExecResult CoreUtility::exec(const char *cmd, bool wait_for_result) {
    ExecResult result;
    char buffer[512];
    try {
        FILE *pipe = popen(cmd, "r");
        if (wait_for_result == false) {
            pclose(pipe);
            result = ExecResult(false, "", "");
            return result;
        }
        // The following can't currently be checked for code coverage as it depends on the
        // device being run on. LCOV_EXCL_START
        if (!pipe) {
            std::string tempstr = "popen() failed with command: " + std::string(cmd);
            pclose(pipe);
            result = ExecResult(true, tempstr, "");
            return result;
        }
        // LCOV_EXCL_STOP
        try {
            while (!feof(pipe)) {
                if (fgets(buffer, 512, pipe) != NULL)
                    result.Result += buffer;
            }
        }
        // The following can't currently be checked for code coverage as it depends on the
        // device being run on. LCOV_EXCL_START
        catch (const std::exception &e) {
            pclose(pipe);
            std::string tempstr = "popen() failed with command: " + std::string(cmd) +
                                  " and exception: " + std::string(e.what());
            result = ExecResult(true, tempstr, "");
            return result;
        }
        // LCOV_EXCL_STOP
        pclose(pipe);
        return result;
    }
    // The following can't currently be checked for code coverage as it depends on the device
    // being run on.
    // LCOV_EXCL_START
    catch (const std::exception &e) {
        std::string tempstr = "popen() failed with command: " + std::string(cmd) +
                              " and exception: " + std::string(e.what());
        result = ExecResult(true, tempstr, "");
        return result;
    }
}
// LCOV_EXCL_STOP

std::string CoreUtility::pretty(eros::file msg) {
    std::string str;
    str = "File: " + msg.file_name;
    str += " Length: " + std::to_string(msg.data_length);
    str += " Extension Type: " + std::to_string(msg.extension_type);
    str += " Status: " + std::to_string(msg.status);
    return str;
}
double CoreUtility::measure_time_diff(ros::Time time_a, ros::Time time_b) {
    double etime = time_a.toSec() - time_b.toSec();
    return etime;
}
}  // namespace eros::eros_utility