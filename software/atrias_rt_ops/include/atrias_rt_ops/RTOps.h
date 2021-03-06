#ifndef RTOPS_H
#define RTOPS_H

/** @file
  * @brief Contains the main class for the RT Operations component.
  */

namespace atrias {
namespace rtOps {
class RTOps;
}
}


#include <sys/mman.h>

// Orocos
#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/Component.hpp>

// ROS
#include <std_msgs/Header.h>

#include <atrias_msgs/log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/rt_ops_event.h>
#include <atrias_shared/globals.h>

// This component (RT Ops)'s includes
#include "atrias_rt_ops/EStopDiags.hpp"
#include "atrias_rt_ops/TimestampHandler.h"
#include "atrias_rt_ops/RobotStateHandler.h"
#include "atrias_rt_ops/ControllerLoop.h"
#include "atrias_rt_ops/OpsLogger.h"
#include "atrias_rt_ops/StateMachine.h"
#include "atrias_rt_ops/RTHandler.h"
#include "atrias_rt_ops/Safety.h"

namespace atrias {

namespace rtOps {

class RTOps : public RTT::TaskContext {
	private:
		RTT::InputPort<uint8_t>                     cManagerDataIn;
		uint8_t                                     cmIn;
		
		/** @brief This is our 1 kHz logging output.
		  */
		RTT::OutputPort<atrias_msgs::log_data>      logCyclicOut;
		
		/** @brief This is our 50 Hz GUI transmission.
		  */
		RTT::OutputPort<atrias_msgs::rt_ops_cycle>  guiCyclicOut;
		
		/** @brief This is the port over which events are sent.
		  */
		RTT::OutputPort<atrias_msgs::rt_ops_event>  eventOut;

		/** @brief This prints out diagnostic information when estops occur
		  */
		EStopDiags*                                 eStopDiags;
		
		/** @brief Handles our timestamps for us
		  */
		TimestampHandler                            timestampHandler;
		
		/** @brief Holds the robot state for us for thread-safety.
		  */
		RobotStateHandler*                          robotStateHandler;
		
		/** @brief Manages the controller loop for us.
		  */
		ControllerLoop*                             controllerLoop;
		
		/** @brief Does our logging for us.
		  */
		OpsLogger                                   opsLogger;
		
		/** @brief Calculates our states for us.
		  */
		StateMachine*                               stateMachine;
		
		/** @brief Handles anything needed to remain realtime safe for us.
		  */
		RTHandler                                   rtHandler;
		
		/** @brief Implements our safety features.
		  */
		Safety*                                     safety;

	public:
		// Constructor
		RTOps(std::string name);
		
		// Grab timestamp.
		uint64_t           getTimestamp();
		
		/** @brief Lets us run the controllers.
		  */
		RTT::OperationCaller<atrias_msgs::controller_output&(atrias_msgs::robot_state&)>
			runController;
		
		/** @brief Lets us send the new controller outputs to the Connector.
		  */
		RTT::OperationCaller<void(atrias_msgs::controller_output)>
			sendControllerOutput;
		
		/** @brief Connects \a runController w/ the top level controller.
		  */
		void connectToController();

		/** @brief Disconnects from a controller.
		  */
		void disconnectController();
		
		/** @brief Allows components to retrieve a ROS Header w/ the right timestamp.
		  * @return A ROS header w/ the right timestamp.
		  */
		std_msgs::Header   getROSHeader();
		
		/** @brief Update the robot state and cycle the controllers.
		  * @param newRobotState The new robot state.
		  * This is called by the communicator when a new robot state is available
		  * The communicator controls the timing of the system through this function.
		  */
		void               newStateCallback(atrias_msgs::robot_state);

		/** @brief Returns a pointer to the EStopDiags instance
		  * @return A pointer to the main EStopDiags instance
		  */
		EStopDiags*        getEStopDiags() const;
		
		/** @brief Grabs a pointer to the RobotStateHandler.
		  * @return A pointer to the RobotStateHandler.
		  */
		RobotStateHandler* getRobotStateHandler();
		
		/** @brief Grabs a pointer to the TimestampHandler.
		  * @return A pointer to the TimestampHandler.
		  */
		TimestampHandler*  getTimestampHandler();
		
		/** @brief Allows other classes to access the OpsLogger.
		  * @return A pointer to the \a OpsLogger.
		  */
		OpsLogger*         getOpsLogger();
		
		/** @brief Allows other classes to access the ControllerLoop.
		  * @return A pointer to the ControllerLoop.
		  */
		ControllerLoop*    getControllerLoop();
		
		/** @brief Allows other classes to access the StateMachine.
		  * @return A pointer to the StateMachine.
		  */
		StateMachine*      getStateMachine();
		
		/** @brief Allows other classes to access the Safety.
		  * @return A pointer to the Safety
		  */
		Safety*            getSafety();
		
		/** @brief Lets Connectors report RT Ops Events.
		  * @param event    The event to be reported.
		  * @param metadata The metadata for this event
		  */
		void               sendEvent(RtOpsEvent event, RtOpsEventMetadata_t metadata);

		// Standard Orocos hooks
		bool               configureHook();
		bool               startHook();
		void               updateHook();
		void               stopHook();
		void               cleanupHook();
};

}

}

#endif // RTOPS_H

// vim: noexpandtab
