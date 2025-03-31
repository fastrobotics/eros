#include <{{cookiecutter.package_name}}/{{cookiecutter.class_name}}/{{cookiecutter.class_name}}.h>
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
bool {{cookiecutter.class_name}}::finish() {
    return true;
}