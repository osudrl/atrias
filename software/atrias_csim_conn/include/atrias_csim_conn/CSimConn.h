#ifndef CSIMCONN_H
#define CSIMCONN_H

/** @file
  * @brief This is the main class for the C++-based simulation connector.
  */

// Orocos
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>

/** @brief This is the amount of torque (in *amps*) needed to hold a hip vertical
  */
#define HIP_HOLD_TORQUE 2.5

/** @brief The hip motor's torque constant... this includes gear ratio!
  */
#define HIP_GEARED_TRQ_CONST 8.35057714286

/** @brief The hip's moment of inertia
  * Not very accurate.
  */
#define HIP_INERTIA 1.0

/** @brief The difference between the hip's vertical and relaxed position
  */
#define HIP_RELAXED_POS_DIFF (10.0 * M_PI / 180.0)

/** @brief The same for extended
  */
#define HIP_EXTENDED_POS_DIFF (20.0 * M_PI / 180.0)

/** @brief This is the current at which a leg will begin to move without external forces.
  */
#define LEG_FRICTION_AMPS 5.0

namespace atrias {

namespace cSimConn {

/** @brief This is the type for \a Hip
  */
typedef uint8_t Hip_t;

/** @brief This represents one hip or another.
  */
enum class Hip: Hip_t {
	LEFT = 0,
	RIGHT
};

/** @brief The type for \a Half.
  */
typedef int8_t Half_t;

/** @brief Represents one half of a leg or the other.
  */
enum class Half: Half_t {
	A = 0,
	B
};

class CSimConn : public RTT::TaskContext {
	private:
		/** @brief This holds the current controller output.
		  */
		atrias_msgs::controller_output cOut;

		/** @brief By calling this, we cycle RT Ops.
		  */
		RTT::OperationCaller<void(atrias_msgs::robot_state)>
			newStateCallback;

		/** @brief This stores the current robot state.
		  */
		atrias_msgs::robot_state robotState;

		/** @brief This simulates one hip.
		  * @param hip      The hip to be simulated.
		  * @param whichHip Whether this is the left or right hip.
		  * @return         The new hip state.
		  */
		atrias_msgs::robot_state_hip simHip(atrias_msgs::robot_state_hip& hip, Hip whichHip);

		/** @brief This simulates one leg half.
		  * @param legHalf The leg half to be simulated.
		  * @param current The commanded current for this legHalf
		  * @param half    The half of the leg this is.
		  * @return        The new leg half.
		  */
		atrias_msgs::robot_state_legHalf simLegHalf(atrias_msgs::robot_state_legHalf& legHalf, double current, Half half);
	
	public:
		/** @brief Initializes the Sim Connector
		  * @param name The name for this component.
		  */
		CSimConn(std::string name);
		
		/** @brief Called by RT Ops w/ update controller torques.
		  * @param controller_output The new controller output.
		  */
		void sendControllerOutput(atrias_msgs::controller_output controller_output);
		
		/** @brief Configures this component.
		  * Run by Orocos.
		  */
		bool configureHook();

		/** @brief Called periodically by Orocos; runs the sim.
		  */
		void updateHook();
};

}

}

#endif // CSIMCONN_H

// vim: noexpandtab
