#include <eros_window/BaseWindow.h>
namespace eros_window {
bool BaseWindow::update(double /* dt */, double t_ros_time) {
    t_ros_time_ = t_ros_time;
    return true;
}
void BaseWindow::update_record_count(uint16_t count) {
    record_count = count;
    record_selected = 0;
}
void BaseWindow::decrement_selected_record() {
    auto _current_record = record_selected;
    _current_record--;
    if (_current_record < 0) {
        _current_record = 0;
    }
    record_selected = _current_record;
}
void BaseWindow::increment_selected_record() {
    auto _current_record = record_selected;
    _current_record++;
    if (_current_record >= (record_count - 1)) {
        _current_record = (record_count - 1);
    }
    record_selected = _current_record;
}
}  // namespace eros_window