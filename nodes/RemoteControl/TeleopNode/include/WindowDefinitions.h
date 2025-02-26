/*! \file WindowDefinitions.h
 */
#pragma once
#include <eros/eros_Definitions.h>
#include <eros_window/CommonWindowDefinitions.h>

//! Namespace for RemoteControl
namespace eros_nodes::RemoteControl {
const double COMMTIMEOUT_THRESHOLD = 5.0f;
/**
 * @brief Commands supported by Windows
 *
 */
enum class WindowCommandType {
    UNKNOWN = 0,    /*!< Uninitialized value. */
    END_OF_LIST = 3 /*!< Last item of list. Used for Range Checks. */
};
/**
 * @brief WindowCommand container
 *
 */
struct WindowCommand {
    WindowCommand() : type(WindowCommandType::UNKNOWN) {
    }
    WindowCommandType type;
    std::string option;
};
/**
 * @brief Container for results of a Key Event
 *
 */
struct KeyEventContainer {
    WindowCommand command;
    MessageText message;
};
}  // namespace eros_nodes::RemoteControl