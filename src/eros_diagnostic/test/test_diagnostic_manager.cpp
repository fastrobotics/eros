/*! \file test_diagnostic_manager.cpp
 */
#include <eros_diagnostic/Diagnostic.h>
#include <eros_diagnostic/DiagnosticManager.h>
#include <eros_diagnostic/DiagnosticUtility.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
using namespace eros::eros_diagnostic;
TEST(BasicTest, Test_DiagnosticManager) {
    DiagnosticManager diagnostic_manager;
    diagnostic_manager.initialize(Diagnostic("UnitTestDevice",
                                             "UnitTestNode-Diagnostics",
                                             System::MainSystem::SIMROVER,
                                             System::SubSystem::ENTIRE_SYSTEM,
                                             System::Component::ENTIRE_SUBSYSTEM));
    {
        std::vector<DiagnosticType> diag_types;
        diag_types.push_back(DiagnosticType::COMMUNICATIONS);
        diag_types.push_back(DiagnosticType::DATA_STORAGE);
        diag_types.push_back(DiagnosticType::SOFTWARE);
        diag_types.push_back(DiagnosticType::SOFTWARE);  // Add Extra
        EXPECT_TRUE(diagnostic_manager.enable_diagnostics(diag_types));
        printf("Diagnostics:\n");
        printf("%s\n", diagnostic_manager.pretty().c_str());
    }

    {
        Diagnostic root_diag = diagnostic_manager.get_root_diagnostic();

        Diagnostic diag = diagnostic_manager.update_diagnostic(root_diag.device_name,
                                                               DiagnosticType::SENSORS,
                                                               Level::Type::INFO,
                                                               Message::NOERROR,
                                                               "No Error");
        printf("Diag: %s\n", DiagnosticUtility::pretty("\t", diag).c_str());
        EXPECT_TRUE(diag.level > Level::Type::WARN);
    }
    {  // Add a Diagnostic for a Different Device

        Diagnostic diag = diagnostic_manager.update_diagnostic("AnotherDevice",
                                                               DiagnosticType::SOFTWARE,
                                                               Level::Type::INFO,
                                                               Message::NOERROR,
                                                               "No Error");
        printf("Diag: %s\n", DiagnosticUtility::pretty("\t", diag).c_str());
        EXPECT_TRUE(diag.level <= Level::Type::WARN);
        diag = diagnostic_manager.read_diagnostic(DiagnosticType::SOFTWARE);
        EXPECT_TRUE(diag.level <= Level::Type::WARN);
    }
    {
        Diagnostic diag = diagnostic_manager.update_diagnostic(
            DiagnosticType::COMMUNICATIONS, Level::Type::INFO, Message::NOERROR, "Running.");
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        diag = diagnostic_manager.read_diagnostic(DiagnosticType::COMMUNICATIONS);
        EXPECT_TRUE(diag.level <= Level::Type::WARN);
    }
    {
        Diagnostic diag = diagnostic_manager.update_diagnostic(
            DiagnosticType::DATA_STORAGE, Level::Type::INFO, Message::NOERROR, "Running.");
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        diag = diagnostic_manager.read_diagnostic(DiagnosticType::DATA_STORAGE);
        EXPECT_TRUE(diag.level <= Level::Type::WARN);
    }
    {
        Diagnostic diag = diagnostic_manager.update_diagnostic(
            DiagnosticType::SOFTWARE, Level::Type::INFO, Message::NOERROR, "Running.");
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    }
    {
        std::vector<Diagnostic> diagnostics = diagnostic_manager.get_diagnostics();

        for (std::size_t i = 0; i < diagnostics.size(); ++i) {
            printf("Diag: %s\n", DiagnosticUtility::pretty("\t", diagnostics.at(i)).c_str());
            EXPECT_TRUE(diagnostics.at(i).level <= Level::Type::NOTICE);
        }
    }
}
TEST(FailureTests, Test_FailureCases) {
    {
        DiagnosticManager diagnostic_manager;
        std::vector<DiagnosticType> emptyDiagnosticTypeList;
        std::string str = diagnostic_manager.pretty();
        EXPECT_EQ(str, "NO DIAGNOSTICS DEFINED YET.");
        EXPECT_FALSE(diagnostic_manager.enable_diagnostics(emptyDiagnosticTypeList));
    }
    {  // Read Diagnostic for Type that isn't defined yet.
        DiagnosticManager diagnostic_manager;
        auto diag = diagnostic_manager.read_diagnostic(DiagnosticType::DATA_STORAGE);
        EXPECT_TRUE(diag.level >= Level::Type::ERROR);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}