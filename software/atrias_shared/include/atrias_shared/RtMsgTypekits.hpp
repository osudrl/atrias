#ifndef RTMSGTYPEKITS_HPP
#define RTMSGTYPEKITS_HPP

/**
  * @file RtMsgTypekits.hpp
  * @author Ryan Van Why
  * @brief This class contains a helper function for registering typekits for ROS messages
  * Any packages that use this need to depend on rtt_rosnode!
  */

// Standard library
#include <string> // For typekit names.

// Orocos
#include <ros_msg_transporter.hpp>        // For registering ROS message transport types
#include <rtt/os/oro_allocator.hpp>       // For the realtime-safe allocator
#include <rtt/types/TemplateTypeInfo.hpp> // Allows us to create typekits

// Namespaces we're inside
namespace atrias {
namespace shared {

class RtMsgTypekits {
	public:
		/**
		  * @brief This registers the typekit for a given ROS type.
		  * @param name A unique name for this type, such as "rt_ops_event_"
		  */
		template <template<class> class msgType>
		static void registerType(const char *name);
};

// Template definitions
template <template<class> class msgType>
void RtMsgTypekits::registerType(const char *name) {
	// This is taken off the Orocos mailinglist
	// http://www.orocos.org/forum/orocos/orocos-users/cannot-transport-ros-message-rttosrtallocator
	RTT::types::Types()->addType(new RTT::types::TemplateTypeInfo<msgType<RTT::os::rt_allocator<uint8_t>>, false>(name));
	RTT::types::Types()->type(name)->addProtocol(3, new ros_integration::RosMsgTransporter<msgType<RTT::os::rt_allocator<uint8_t>>>());
}

// End namespaces
}
}

#endif // RTMSGTYPEKITS_HPP

// Tab-based indentation
// vim: noexpandtab
