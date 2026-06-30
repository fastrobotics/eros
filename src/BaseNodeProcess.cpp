#include <eros/BaseNodeProcess.h>
namespace eros {
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
}