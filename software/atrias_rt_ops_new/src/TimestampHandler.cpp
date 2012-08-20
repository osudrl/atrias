#include "atrias_rt_ops/TimestampHandler.h"

namespace atrias {

namespace rtOps {

TimestampHandler::TimestampHandler() {
	timestamp = 0;
}

void TimestampHandler::setTimestamp(std_msgs::Header &newTimestamp) {
	RTT::os::MutexLock lock(timestampLock);
	timestamp = SECOND_IN_NANOSECONDS * newTimestamp.stamp.sec +
	                                    newTimestamp.stamp.nsec;
}

RTT::os::TimeService::nsecs TimestampHandler::getTimestamp() {
	RTT::os::MutexLock lock(timestampLock);
	return timestamp;
}

std_msgs::Header TimestampHandler::getTimestampHeader() {
	std_msgs::Header out;
	RTT::os::MutexLock lock(timestampLock);
	out.stamp.nsec = timestamp % SECOND_IN_NANOSECONDS;
	out.stamp.sec  = timestamp / SECOND_IN_NANOSECONDS;
	return out;
}

}

}
