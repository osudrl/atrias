#ifndef OPSLOGGER_H
#define OPSLOGGER_H

/** @file
  * @brief Contains the main logging infrastructure for RT Ops.
  */

#include <stdint.h>

// Orocos
#include <rtt/OutputPort.hpp>
#include <rtt/os/TimeService.hpp>

#include <atrias_shared/globals.h>
#include <atrias_shared/GuiPublishTimer.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/rt_ops_cycle.h>
#include <atrias_msgs/rt_ops_event.h>

namespace atrias {

namespace rtOps {

class OpsLogger {
	/** @brief A pointer to the port used to log RT Ops's cyclic data.
	  */
	RTT::OutputPort<atrias_msgs::rt_ops_cycle>* logCyclicOut;
	
	/** @brief A pointer to the port used to send RT Ops's cyclic data to the GUI.
	  */
	RTT::OutputPort<atrias_msgs::rt_ops_cycle>* guiCyclicOut;
	
	/** @brief A pointer to the port we use to send events.
	  */
	RTT::OutputPort<atrias_msgs::rt_ops_event>* eventOut;
	
	/** @brief Does the timing for the 50 Hz stream.
	  */
	shared::GuiPublishTimer                     guiPublishTimer;
	
	/** @brief Stores this cycle's RT Ops Cycle message.
	  */
	atrias_msgs::rt_ops_cycle                   rtOpsCycle;
	
	public:
		/** @brief Initializes the OpsLogger.
		  * @param log_robot_state_out A pointer to the robot state logging output port.
		  * @param gui_robot_state_out A pointer to the 50Hz robot state output port.
		  */
		OpsLogger(RTT::OutputPort<atrias_msgs::rt_ops_cycle>* log_cyclic_out,
		          RTT::OutputPort<atrias_msgs::rt_ops_cycle>* gui_cyclic_out,
		          RTT::OutputPort<atrias_msgs::rt_ops_event>* event_out);
		
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
		
		/** @brief Send out an RT Ops event.
		  * @param error    The specific event to be reported.
		  * @param metadata The metadata associated with this event.
		  */
		void sendEvent(RtOpsEvent event, RtOpsEventMetadata_t metadata=0);
};

}

}

#endif // OPSLOGGER_H

// vim: noexpandtab
