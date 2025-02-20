/*! \file test_base.cpp
 */
#include <eros/BaseNodeProcess.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
using namespace eros::eros_diagnostic;
static std::string get_hostname() {
    char name[1024];
    name[1023] = '\0';
    gethostname(name, 1023);
    return std::string(name);
}
class BaseNodeProcessTester : public BaseNodeProcess
{
   public:
    BaseNodeProcessTester() {
    }
    ~BaseNodeProcessTester() {
    }

    eros_diagnostic::Diagnostic finish_initialization() {
        eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
        return diag;
    }

    void reset() {
    }
    void cleanup() {
        base_cleanup();
    }

    eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time) {
        eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
        diag = base_update(t_dt, t_ros_time);
        return diag;
    }
    std::vector<eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg) {
        eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
        std::vector<eros_diagnostic::Diagnostic> diag_list;
        (void)msg;
        return diag_list;
    }
    std::vector<eros_diagnostic::Diagnostic> check_programvariables() {
        eros_diagnostic::Diagnostic diag = diagnostic_manager.get_root_diagnostic();
        std::vector<eros_diagnostic::Diagnostic> diag_list;
        return diag_list;
    }
    std::string pretty() {
        return "";
    }
};
TEST(BasicTest, TestOperation_BaseNodeProcess) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    json json_obj = tester->read_configuration(
        "", true, std::string(ZIPFILETESTDATA_DIR) + "/catkin_ws/src/sconfig/DeviceList.json");
    printf("host: %s\n", tester->get_hostname().c_str());
    EXPECT_TRUE(json_obj.size() > 0);
    printf("json:%s\n", json_obj.dump().c_str());
    EXPECT_TRUE(tester->update(0.0, 0.0).level <= Level::Type::NOTICE);
    tester->cleanup();
    delete logger;
    delete tester;
}
TEST(BasicTest, SupportFunctions) {
    {  // Test File Packaging
        FileHelper::FileInfo fileInfo;
        fileInfo = BaseNodeProcess::read_file("AFilethatwillNeverExist.zip");
        EXPECT_TRUE(fileInfo.byte_size == 0);
        EXPECT_TRUE(fileInfo.fileType == FileHelper::FileType::ZIP);
        EXPECT_TRUE(fileInfo.fileStatus == FileHelper::FileStatus::FILE_ERROR);

        fileInfo = BaseNodeProcess::read_file(std::string(ZIPFILETESTDATA_DIR) + "/zip/Zip1.zip");
        EXPECT_TRUE(fileInfo.byte_size > 0);
        EXPECT_TRUE(fileInfo.fileType == FileHelper::FileType::ZIP);
        EXPECT_TRUE(fileInfo.fileStatus == FileHelper::FileStatus::FILE_OK);
        EXPECT_TRUE(fileInfo.file_name.size() > 0);
        EXPECT_TRUE(fileInfo.folder.size() > 0);
        EXPECT_TRUE(fileInfo.full_path.size() > 0);

        fileInfo =
            BaseNodeProcess::write_file("~/test/Zip2.zip", fileInfo.data, fileInfo.byte_size);
        EXPECT_TRUE(fileInfo.fileStatus == FileHelper::FileStatus::FILE_OK);
    }
}
TEST(BasicTest, TestOperation_StateChange) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    // Sequence through States
    EXPECT_EQ(tester->get_nodestate(), Node::State::START);
    EXPECT_FALSE(tester->request_statechange(Node::State::RUNNING));  // Not Allowed
    eros_diagnostic::Diagnostic diag = tester->update(0.0, 0.0);
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZING);
    EXPECT_FALSE(tester->request_statechange(Node::State::RUNNING));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZING);

    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZED));
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZED);
    EXPECT_FALSE(tester->request_statechange(Node::State::INITIALIZING));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZED);

    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);

    EXPECT_TRUE(tester->request_statechange(Node::State::PAUSED));
    EXPECT_FALSE(tester->request_statechange(Node::State::INITIALIZING));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::PAUSED);

    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING));
    EXPECT_FALSE(tester->request_statechange(Node::State::INITIALIZING));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);

    EXPECT_TRUE(tester->request_statechange(Node::State::RESET));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RESET);
    EXPECT_FALSE(tester->request_statechange(Node::State::START));  // Not Allowed
    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZING));
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZING);
    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZED));

    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);
    EXPECT_TRUE(tester->request_statechange(Node::State::RESET));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RESET);
    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);

    EXPECT_TRUE(tester->request_statechange(Node::State::FINISHED));
    EXPECT_FALSE(tester->request_statechange(Node::State::START));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::FINISHED);

    delete logger;
    delete tester;
}
TEST(BasicTest, TestOperation_StateChange_Override) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    // Sequence through States
    EXPECT_EQ(tester->get_nodestate(), Node::State::START);
    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);

    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZING, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZING);

    EXPECT_TRUE(tester->request_statechange(Node::State::PAUSED, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::PAUSED);

    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZED, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZED);

    EXPECT_TRUE(tester->request_statechange(Node::State::RESET, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RESET);

    EXPECT_TRUE(tester->request_statechange(Node::State::START, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::START);

    EXPECT_TRUE(tester->request_statechange(Node::State::FINISHED, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::FINISHED);

    EXPECT_TRUE(tester->request_statechange(Node::State::START, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::START);

    delete logger;
    delete tester;
}

TEST(ConfigurationTests, ConfigurationTestCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    {  // Failure Tests
        logger->log_warn("Testing Failure Cases...");
        std::string FilePathOfZeroLength = "";
        json empty;
        json obj;
        obj = tester->read_configuration("abcd", true, FilePathOfZeroLength);
        EXPECT_EQ(obj, empty);

        std::string AFilePathThatWillNeverExist = "/AFilePathThatWillNeverExist";
        obj = tester->read_configuration("abcd", true, AFilePathThatWillNeverExist);
        EXPECT_EQ(obj, empty);
    }
    {
        std::string ConfigPath =
            std::string(getenv("HOME")) + "/catkin_ws/src/config/DeviceList.json";
        json empty;
        json obj;

        obj = tester->read_configuration(tester->get_hostname(), true, ConfigPath);
        EXPECT_NE(obj, empty);
    }
    delete tester;
    delete logger;
}
TEST(FileReadTests, FileReadTestCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    logger->log_warn("Testing Failure Cases...");
    {
        // Failure Tests: Unsupported File Types
        std::string ConfigPath =
            std::string(getenv("HOME")) +
            "/catkin_ws/src/config/DeviceList.json";  // File Exists, but this function isn't
                                                      // intended to support.
        FileHelper::FileInfo info = tester->read_file(ConfigPath);
        EXPECT_EQ(info.fileStatus, FileHelper::FileStatus::FILE_ERROR);
        EXPECT_EQ(info.fileType, FileHelper::FileType::UNKNOWN);
    }
    {
        // Failure Tests: Corrupted Files
        std::string filePath = "EmptyZip.zip";
        std::string cmd = "touch " + filePath;
        ExecResult res = eros_utility::CoreUtility::exec(cmd.c_str(), true);
        EXPECT_FALSE(res.AnyError);
        FileHelper::FileInfo info = tester->read_file(filePath);
        EXPECT_EQ(info.fileStatus, FileHelper::FileStatus::FILE_ERROR);
        EXPECT_EQ(info.fileType, FileHelper::FileType::ZIP);
        cmd = "rm " + filePath;
        res = eros_utility::CoreUtility::exec(cmd.c_str(), true);
        EXPECT_FALSE(res.AnyError);
    }
    delete tester;
    delete logger;
}
TEST(FileWriteTests, FileWriteTestCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    logger->log_warn("Testing Failure Cases...");
    {
        // Failure Tests: Unsupported File Types
        std::string validFile = "TestFile.txt";
        char someData[7] = "Hello";
        FileHelper::FileInfo info = tester->write_file(validFile, someData, 1);
        EXPECT_EQ(info.fileStatus, FileHelper::FileStatus::FILE_ERROR);
        EXPECT_EQ(info.fileType, FileHelper::FileType::UNKNOWN);

        std::string inValidFile = "/AFileThatShouldNeverExist.zip";
        info = tester->write_file(inValidFile, someData, 1);
        EXPECT_EQ(info.fileStatus, FileHelper::FileStatus::FILE_ERROR);
        EXPECT_EQ(info.fileType, FileHelper::FileType::ZIP);
    }

    delete tester;
    delete logger;
}
TEST(DirectoryReadTests, DirectoryReadTestsCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros_diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    {
        std::vector<std::string> fileList = tester->get_files_indir(std::string(getenv("HOME")));
        EXPECT_GT(fileList.size(), 0);
    }

    delete tester;
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}