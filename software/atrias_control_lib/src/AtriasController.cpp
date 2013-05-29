#include "atrias_control_lib/AtriasController.hpp"

namespace atrias {
namespace controller {

AtriasController::AtriasController(const AtriasController * const parent,
                                   const std::string              &name) :
	name(std::string(parent->getName()) + "_" + name),
	tlc(parent->getTLC())
{
	// All the magic happens above.
}

AtriasController::AtriasController(const std::string &name) :
	name(name),
	tlc(*this)
{
	// This space intentionally left blank
}

double AtriasController::clamp(double num, double a, double b) {
	auto min = std::min(a, b);
	auto max = std::max(a, b);

	if (num < min)
		return min;

	if (max < num)
		return max;

	return num;
}

const std::string& AtriasController::getName() const {
	return this->name;
}

//const std_msgs::Header_<RTT::os::rt_allocator<uint8_t>>& AtriasController::getROSHeader() const {
const std_msgs::Header& AtriasController::getROSHeader() const {
	// This is overridden by the ATC class, preventing recursion.
	return tlc.getROSHeader();
}

RTT::TaskContext& AtriasController::getTaskContext() const {
	// The ATC class overrides this function, so this is not actually
	// recursive.
	return tlc.getTaskContext();
}

AtriasController& AtriasController::getTLC() const {
	return this->tlc;
}

}
}

// vim: noexpandtab
