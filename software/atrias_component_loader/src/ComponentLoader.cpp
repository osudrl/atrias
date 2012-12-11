#include "atrias_component_loader/ComponentLoader.hpp"

namespace atrias {
namespace ComponentLoader {

RTT::TaskContext* ComponentLoader::getTaskContext() {
	log(RTT::Info) << "getService" << RTT::endlog();
}

ComponentLoader::ComponentLoader(RTT::TaskContext* task_context, std::string type) {
	log(RTT::Info) << "loadComponentLoader" << RTT::endlog();
}

ComponentLoader::~ComponentLoader() {
	log(RTT::Info) << "destructor" << RTT::endlog();
}

}
}

// vim: noexpandtab
