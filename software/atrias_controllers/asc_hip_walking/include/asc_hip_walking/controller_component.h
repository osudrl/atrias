#ifndef __ASC_HIP_WALKING_H__
#define __ASC_HIP_WALKING_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for the asc_hip_walking subcontroller.
 */

// Orocos
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>

#include <robot_invariant_defs.h>

// Datatypes
#include <asc_hip_walking/controller_log_data.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_hip_walking;

namespace atrias {
namespace controller {

class ASCHipWalking : public TaskContext {
	private:
		/** @brief Computes the correct hip angle given the leg angle and boom angle.
		  * @param legAngle This leg's angle
		  * @param boomAngle The boom angle.
		  * @param hip Which hip -- 0 for left, 1 for right.
		  * @return The desired hip angle.
		  */
		double runController(double legAngle, double boomAngle, int hip);
		
		// Logging
		OutputPort<controller_log_data> logPort;
		
	public:
		// Constructor
		ASCHipWalking(std::string name);
		
		// Standard Orocos hooks
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}
}

#endif
