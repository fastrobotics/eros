#include "{{cookiecutter.class_name}}.h"
namespace {{cookiecutter.package_name}} {
bool {{cookiecutter.class_name}}::init(eros::Logger* _logger) {
    logger = _logger;
    return reset();
}
bool {{cookiecutter.class_name}}::reset() {
    run_time = 0.0;
    return true;
}
bool {{cookiecutter.class_name}}::update(double dt) {
    run_time += dt;
    return true;
}
std::string {{cookiecutter.class_name}}::pretty() {
    std::string str = "{{cookiecutter.class_name}}";
    return str;
}
bool {{cookiecutter.class_name}}::finish() {
    return true;
}
}  // namespace {{cookiecutter.package_name}}