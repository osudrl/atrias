#include "atrias_rt_ops/RTHandler.h"

namespace atrias {

namespace rtOps {

void signal_handler (int signum) {
	return;
}

RTHandler::RTHandler() {
	signal(SIGXCPU, signal_handler);
}

void RTHandler::beginRT() {
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		log(RTT::Warning) << "[RTOps] Failed to lock memory!" << RTT::endlog();
	}
}

void RTHandler::endRT() {
	if (munlockall() == -1) {
		log(RTT::Error) << "[RTOps] Failed to unlock memory!" << RTT::endlog();
	}
}

}

}
