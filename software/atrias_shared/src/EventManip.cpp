#include "atrias_shared/EventManip.hpp"

namespace atrias {

namespace rtOps {

atrias_msgs::rt_ops_event buildEvent(RtOpsEvent eventType) {
	atrias_msgs::rt_ops_event event;
	event.event = (RtOpsEvent_t) eventType;
	return event;
}

}

}
