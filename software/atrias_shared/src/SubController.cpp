#include "atrias_shared/SubController.hpp"

namespace atrias {
namespace controller{

RTT::Service::shared_ptr SubController::getService(std::string name) {
	log(RTT::Info) << "getService" << RTT::endlog();
}

SubController::SubController(RTT::TaskContext* task_context, std::string type) {
	log(RTT::Info) << "loadController" << RTT::endlog();
}

SubController::~SubController() {
	log(RTT::Info) << "destructor" << RTT::endlog();
}

}
}

// vim: noexpandtab
