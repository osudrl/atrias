#ifndef __ASC_HIP_FORCE_H__
#define __ASC_HIP_FORCE_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for the asc_hip_force subcontroller.
 */

// Orocos
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>

// Our stuff
#include <asc_hip_force/controller_log_data.h>
#include <atrias_asc_loader/ASCLoader.hpp>
#include <robot_invariant_defs.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_hip_force;

namespace atrias {
namespace controller {

class ASCHipForce : public TaskContext {
	private:
		// Operations
		double runController(uint16_t toeSwitch, int32_t kneeForce, double legBodyAngle, double legBodyVelocity);
		// This should be run after \a runController()
		bool   getOnGround();

		// Our gains
		double flightP;
		double flightD;
		double stanceP;
		double stanceD;

		// And gains for the toe decode controller
		double toeFilterGain;
		double toeThreshold;

		// Properties allowing us to set the PD controller's gains.
		Property<double> P;
		Property<double> D;

		// And the same for the toe decode controller
		Property<double> toeFilterGainProperty;
		Property<double> toeThresholdProperty;

		ASCLoader pdLoader;
		ASCLoader toeLoader;

		// Whether or not we're in contact with the ground.
		bool onGround;

		OperationCaller<bool(uint16_t)> runToeDecode;
		OperationCaller<double(double, double, double, double)> runPD;

	public:
		// Constructor
		ASCHipForce(std::string name);

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

// vim: noexpandtab
