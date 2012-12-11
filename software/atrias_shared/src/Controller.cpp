#include "atrias_shared/Controller.hpp"

namespace atrias {
namespace controller {

RTT::TaskContext* Controller::getTaskContext() {
	log(RTT::Info) << "getService" << RTT::endlog();
}

Controller::Controller(RTT::TaskContext* task_context, std::string type) {
	log(RTT::Info) << "loadController" << RTT::endlog();
}

Controller::~Controller() {
	log(RTT::Info) << "destructor" << RTT::endlog();
}

}
}

// vim: noexpandtab
