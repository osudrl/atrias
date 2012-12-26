#ifndef CMCOMMS_H
#define CMCOMMS_H

/** @file CMComms.h
  * @brief This class is responsible for communications to and from the Controller Manager
  */

namespace atrias {
namespace rtOps {
class CMComms;
}
}

// Orocos
#include <rtt/InputPort.hpp>

// Our stuff
#include <atrias_rt_ops/RTOps.h>
#include <atrias_shared/EventManip.hpp>
#include <atrias_shared/globals.h>

namespace atrias {

namespace rtOps {

class CMComms {
	private:
		/** @brief A pointer to the RT Ops instance.
		  */
		RTOps*                                                      rtOps;

		/** @brief This is the port over which the Controller Manager sends commands.
		  */
		RTT::InputPort<controllerManager::ControllerManagerCommand> cmInPort;

		/** @brief This stores the Controller Manager's commands.
		  */
		controllerManager::ControllerManagerCommand                 cmIn;
	
	public:
		/** @brief Initializes this CMComms instance.
		  * @param rt_ops A pointer to the main RT Ops instance.
		  */
		CMComms(RTOps *rt_ops);

		/** @brief Checks for a new command from the Controller Manager.
		  * @return Whether or not there was a new command from the Controller Manager.
		  */
		bool updateHook();
};

}

}

#endif // CMCOMMS_H
