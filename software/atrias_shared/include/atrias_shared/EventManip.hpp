#ifndef EVENTMANIP_HPP
#define EVENTMANIP_HPP

/** @file EventManip.hpp
  * @brief Contains functions for manipulating events (and their metadata).
  */

// Our stuff
#include <atrias_msgs/rt_ops_event.h>
#include "atrias_shared/globals.h"

namespace atrias {

namespace rtOps {

/** @brief This creates an event without metadata.
  * @param eventType The type of event to be created.
  */
atrias_msgs::rt_ops_event buildEvent(RtOpsEvent eventType);

/** @brief This creates an event of the given type and populates its metadata
  */
template<class T> atrias_msgs::rt_ops_event buildEventMetadata(RtOpsEvent eventType, T metadata) {
	return buildEventPtr(eventType, &metadata);
}

/** @brief This is just buildMetadata, but it takes in a pointer, instead of the metadata itself.
  * This may be useful for building an event with the metadata in a struct.
  */
template<class T> atrias_msgs::rt_ops_event buildEventPtr(RtOpsEvent eventType, T* metadata) {
	atrias_msgs::rt_ops_event event = buildEvent(eventType);
	
	// Populate the metadata
	for (size_t i = 0; i < sizeof(T); i++) {
		event.metadata.push_back( ((uint8_t*) metadata)[i] );
	}
	return event;
}

/** @brief This reads in the metadata from an event.
  * @param event The event
  * @return This event's metadata.
  */
template<class T> T readEventMetadata(atrias_msgs::rt_ops_event& event) {
	return *((T*) event.metadata.data());
}

}

}

#endif // EVENTMANIP_HPP

// vim: noexpandtab
