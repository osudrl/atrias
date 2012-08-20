/*! \file atrias_ecat_master.cpp
 *  \author Soo-Hyun Yoo
 *  \brief ATRIAS EtherCAT Master.
 */

#ifndef RTOPS_H
#define RTOPS_H

class RTOps;

// Orocos
#include <rtt/os/main.h>

#include <rtt/RTT.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>

#include <rtt/os/Semaphore.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>

// ROS
#include <ros/ros.h>
#include "std_msgs/Header.h"

// C
#include <stdlib.h>
#include <sys/mman.h>
#include <signal.h>

// ATRIAS
#include <atrias_shared/globals.h>
#include <atrias_shared/drl_math.h>

#include <robot_invariant_defs.h>
#include <atrias_msgs/rt_ops_cycle.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/GuiPublishTimer.h>
#include <atrias_msgs/rt_ops_status.h>  // Output to controller manager
#include <atrias_msgs/atrias_debug.h>   // Debug stream.
// RT Ops component internals
#include "atrias_rt_ops/Communicator.h"
#include "atrias_rt_ops/ECatComm.h"
//#include "atrias_rt_ops/GazeboConnector.h"
#include "atrias_rt_ops/NoopConnector.h"

using namespace RTT;
using namespace Orocos;
using namespace atrias::controllerManager;
using namespace atrias::shared;
using namespace std_msgs;

class RTOps : public RTT::TaskContext, public RTT::Activity {
	private:
		/** @brief Used to signal the controller execution thread. */
		RTT::os::Semaphore                              signal;
		
		/** @brief Used to synchronize access to the logging timestamp.
		  */
		RTT::os::Mutex                                  logTimestampLock;

		/** @brief The current logging timestamp value.
		  * This is copied from robotState right before running the controllers.
		  */
		RTT::os::TimeService::nsecs                     logTimestamp;
		
		OutputPort<atrias_msgs::rt_ops_status>          cManagerDataOut;
		InputPort<uint8_t>                              cManagerDataIn;
		uint8_t                                         cmIn;

        OutputPort<atrias_msgs::rt_ops_cycle>           guiDataOut;
		OutputPort<atrias_msgs::rt_ops_cycle>           logDataOut;

		Communicator*                                   comm;
		int                                             counter;
		
        /** @brief Used to instruct the controller loop to shut down.
          */
		bool                                            stop;

		/** @brief Command the controller loop to run or not run the controller.
		  */
		bool                                            controllerLoaded;
		
		/** @brief Used to protect access to controllerLoaded.
		  * So the controller won't run while controllerLoaded == false.
		  */
		RTT::os::Mutex                                  controllerLoadedLock;

		void loadController();
		
		/** @brief Causes the controller thread to cycle once (once more
		  * if it's already running).
		  */
		void                                            cycleControlThread();
	public:
		RTT::os::TimeService::nsecs                     currentTimestamp;
		
		RTT::os::Mutex                                  bigComponentLock;
		RTT::os::Mutex                                  robotStateLock;
		RTT::os::Mutex                                  cmOutLock;
		atrias_msgs::rt_ops_cycle                       rtOpsCycle;
		atrias_msgs::controller_output                  controllerOutput;
		atrias_msgs::rt_ops_status                      cmOut;
	    GuiPublishTimer                                 *guiPubTimer;
        GuiPublishTimer                                 *controllerManagerPubTimer;

		// Get controller output from a top-level controller.
		OperationCaller<atrias_msgs::controller_output(atrias_msgs::robot_state)>
		    runController;

        // Grab timestamp.
        uint64_t getTimestamp();
        Header getRosHeader();
        Header getInternalRosHeader();

		// The callback for the Communicator.
		void newStateCallback();

		// Sends the updated robot state to the GUI and log
		void sendRobotState();
		
		void zeroMotorTorques();

		// Constructor
		RTOps(std::string name);

/*		InputPort<atrias_msgs::robot_state> gazeboDataIn;
		OutputPort<atrias_msgs::controller_output> gazeboDataOut;*/

		// Standard Orocos hooks
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		
		// For the controller loop
		/** @brief Cycles the controller.
		  * It will continue running if the controller is missing cycles.
		  */
		void loop();
		/** @brief This should stop the controller loop.
		  */
		bool breakLoop();
};

#endif // RTOPS_H
