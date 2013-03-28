#include "atrias_lib_control/AtriasController.hpp"

namespace atrias {
namespace controller {

AtriasController::AtriasController(const AtriasController &parent,
                                   const std::string      &name) :
	name(std::string(parent.getName()).append(name)),
	tc(parent.getTaskContext())
{
	// All the magic happens above.
}

const std::string& AtriasController::getName() const {
	return this->name;
}

RTT::TaskContext& AtriasController::getTaskContext() const {
	return this->tc;
}

}
}

// vim: noexpandtab
