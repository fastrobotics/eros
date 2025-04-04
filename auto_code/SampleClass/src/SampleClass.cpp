#include "SampleClass.h"
namespace SamplePackage {
bool SampleClass::init(eros::Logger* _logger) {
    logger = _logger;
    return reset();
}
bool SampleClass::reset() {
    run_time = 0.0;
    return true;
}
bool SampleClass::update(double dt) {
    run_time += dt;
    return true;
}
std::string SampleClass::pretty() {
    std::string str = "SampleClass";
    return str;
}
bool SampleClass::finish() {
    return true;
}
}  // namespace SamplePackage