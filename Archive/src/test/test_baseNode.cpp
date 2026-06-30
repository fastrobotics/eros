#include <eros/BaseNode.h>
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <map>
using namespace eros;
std::string robot_namespace = "/test";
class BaseNodeTester : public eros::BaseNode
{
   public:
    const std::string BASE_NODE_NAME = "test_baseNode";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 1;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 1;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 22-Feb-2021";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::ENTIRE_SYSTEM;

    /*! \brief What Component this Node falls under.*/
    const eros::System::Component DIAGNOSTIC_COMPONENT = eros::System::Component::CONTROLLER;
    bool start() {
        set_no_launch_enabled(true);
        set_robotnamespace(robot_namespace);

        disable_device_client = true;
        initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
        bool status = false;
        set_basenodename(BASE_NODE_NAME);
        initialize_firmware(
            MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
        // enable_ready_to_arm_pub(true);
        diagnostic = preinitialize_basenode();
        if (diagnostic.level > Level::Type::WARN) {
            return false;
        }
        diagnostic = read_launchparameters();
        if (diagnostic.level > Level::Type::WARN) {
            return false;
        }
        disable_armedstate_sub();
        set_loop1_rate(1.0);
        set_loop2_rate(2.0);
        set_loop3_rate(3.0);
        set_ros_rate(30.0);
        std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
        diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
        diagnostic_types.push_back(eros_diagnostic::DiagnosticType::DATA_STORAGE);
        diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
        deviceInfo.received = true;

        diagnostic = finish_initialization();
        if (diagnostic.level > Level::Type::WARN) {
            return false;
        }
        if (diagnostic.level < Level::Type::WARN) {
            diagnostic.type = eros_diagnostic::DiagnosticType::SOFTWARE;
            diagnostic.level = Level::Type::INFO;
            diagnostic.message = eros_diagnostic::Message::NOERROR;
            diagnostic.description = "Node Configured.  Initializing.";
            get_logger()->log_diagnostic(diagnostic);
        }
        status = true;
        return status;
    }
    bool run_loop1() {
        return true;
    }
    bool run_loop2() {
        return true;
    }
    bool run_loop3() {
        return true;
    }
    bool run_001hz() {
        return true;
    }
    bool run_01hz() {
        return true;
    }
    bool run_1hz() {
        return true;
    }
    bool run_10hz() {
        return true;
    }
    bool run_01hz_noisy() {
        return true;
    }
    void thread_loop() {
    }
    bool changenodestate_service(eros::srv_change_nodestate::Request&,
                                 eros::srv_change_nodestate::Response&) {
        return true;
    }
    void command_Callback(const eros::command::ConstPtr&) {
    }
    void cleanup() {
    }

    eros_diagnostic::Diagnostic read_launchparameters() {
        eros_diagnostic::Diagnostic diag = diagnostic;
        get_logger()->log_notice("Configuration Files Loaded.");
        return diag;
    }
    eros_diagnostic::Diagnostic finish_initialization() {
        eros_diagnostic::Diagnostic diag = diagnostic;
        resource_available_monitor =
            new ResourceMonitor(node_name, ResourceMonitor::Mode::DEVICE, diag, logger);
        diag = resource_available_monitor->init();
        if (diag.level > Level::Type::WARN) {
            logger->log_diagnostic(diag);
            return diag;
        }
        return diag;
    }
    std::string pretty() {
        std::string str = "";
        return str;
    }

   private:
    eros::ResourceMonitor* resource_available_monitor;
};
TEST(BaseNode, BasicFunctionality) {
    BaseNodeTester tester;
    bool status = tester.start();
    printf("Verbosity Level: %s\n", tester.get_verbositylevel().c_str());
    EXPECT_TRUE(status);
    {                                                         // Namespace checks
        std::map<std::string, std::string> namespace_checks;  // First=Input, Second=Output
        namespace_checks.insert(std::pair<std::string, std::string>("", "/"));
        namespace_checks.insert(std::pair<std::string, std::string>("/", "/"));
        namespace_checks.insert(std::pair<std::string, std::string>("//", "/"));
        namespace_checks.insert(std::pair<std::string, std::string>("1", "/1/"));
        namespace_checks.insert(std::pair<std::string, std::string>("/1/", "/1/"));
        namespace_checks.insert(std::pair<std::string, std::string>("1/", "/1/"));
        namespace_checks.insert(std::pair<std::string, std::string>("1/", "/1/"));
        namespace_checks.insert(std::pair<std::string, std::string>("/1/2/", "/1/2/"));
        namespace_checks.insert(std::pair<std::string, std::string>("1/2/", "/1/2/"));
        namespace_checks.insert(std::pair<std::string, std::string>("/1/2", "/1/2/"));
        namespace_checks.insert(std::pair<std::string, std::string>(
            "aabccasd/2342/3416314//asc//asaa", "/aabccasd/2342/3416314/asc/asaa/"));
        uint16_t i = 0;
        for (std::map<std::string, std::string>::iterator it = namespace_checks.begin();
             it != namespace_checks.end();
             ++it) {
            std::string input_str = it->first;
            std::string expected_str = it->second;
            std::string output_str = BaseNode::validate_robotnamespace(input_str);
            EXPECT_TRUE(output_str == expected_str);
            i++;
        }
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_baseNode");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}