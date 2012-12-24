#ifndef OPSLOGGER_H
#define OPSLOGGER_H

/** @file
  * @brief Contains the main logging infrastructure for RT Ops.
  */

class OpsLogger;

#include <stdint.h>

// Orocos
#include <rtt/OutputPort.hpp>
#include <rtt/os/TimeService.hpp>

#include <atrias_shared/EventManip.hpp>
#include <atrias_shared/GuiPublishTimer.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/rt_ops_cycle.h>
#include <atrias_msgs/rt_ops_event.h>
#include <atrias_rt_ops/RTOps.h>

namespace atrias {

namespace rtOps {

class OpsLogger {
	/** @brief The port used to log RT Ops's cyclic data.
	  */
	RTT::OutputPort<atrias_msgs::rt_ops_cycle> logCyclicOut;
	
	/** @brief The port used to send RT Ops's cyclic data to the GUI.
	  */
	RTT::OutputPort<atrias_msgs::rt_ops_cycle> guiCyclicOut;
	
	/** @brief The port we use to send events.
	  */
	RTT::OutputPort<atrias_msgs::rt_ops_event> eventOut;
	
	/** @brief Does the timing for the 50 Hz stream.
	  */
	shared::GuiPublishTimer                    guiPublishTimer;
	
	/** @brief Stores this cycle's RT Ops Cycle message.
	  */
	atrias_msgs::rt_ops_cycle                  rtOpsCycle;
	
	public:
		/** @brief Initializes the OpsLogger.
		  * @param rt_ops A pointer to the main RT Ops instance.
		  */
		OpsLogger(RTOps *rt_ops);
		
		/** @brief Begins a new cycle. This will send out the rt ops cycle message.
		  */
		void beginCycle();
		
		/** @brief Ends a cycle. This records this cycle's ending timestamp.
		  */
		void endCycle();
		
		/** @brief Logs the new robot state.
		  */
		void logRobotState(atrias_msgs::robot_state& state);
		
		/** @brief Logs the commanded controller output.
		  */
		void logControllerOutput(atrias_msgs::controller_output& output);
		
		/** @brief Logs the clamped controller output.
		  */
		void logClampedControllerOutput(atrias_msgs::controller_output& clamped_output);
		
		/** @brief Transmit an RT Ops event.
		  * @param event The event to be sent.
		  */
		void sendEvent(atrias_msgs::rt_ops_event &event);
};

}

}

#endif // OPSLOGGER_H

// vim: noexpandtab
