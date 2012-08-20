#ifndef TIMESTAMPHANDLER_H
#define TIMESTAMPHANDLER_H

/** @file
  * @brief Protects access to/from the timestamp for logging.
  */

// ROS
#include <std_msgs/Header.h>

// Orocos
#include <rtt/os/TimeService.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>

#include <atrias_shared/globals.h>

namespace atrias {

namespace rtOps {

class TimestampHandler {
	/** @brief Holds the current timestamp value.
	  */
	RTT::os::TimeService::nsecs timestamp;
	
	/** @brief Protects access to the timestamp value.
	  */
	RTT::os::Mutex              timestampLock;
	
	public:
		/** @brief Initializes the object.
		  */
		TimestampHandler();
		
		/** @brief Sets the timestamp value (threadsafe).
		  * @param newTimestamp A ROS header w/ the new timestamp.
		  */
		void setTimestamp(std_msgs::Header &newTimestamp);
		
		/** @brief Gets the timestamp value (threadsafe).
		  */
		RTT::os::TimeService::nsecs getTimestamp();
		
		/** @brief Returns the timestamp value as a ROS header (threadsafe).
		  * @return          The ROS header.
		  */
		std_msgs::Header getTimestampHeader();
};

}

}

#endif // TIMESTAMPHANDLER_H
