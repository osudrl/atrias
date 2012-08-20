#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

/** @file
  * @brief Describes the interface for a Communicator to interface with the
  * Realtime Operations class.
  * Communicators include the EtherCAT Communications class as well as the
  * non-EtherCAT-based simulation connector.
  */

// Orocos
#include <rtt/Activity.hpp>

class Communicator : public RTT::Activity {
	public:
		/** @brief Inits the Communicator
		  * @param loop_period The loop period for the Activity -- 0 for no periodic
		  * activity.
		  */
		Communicator(RTT::Seconds loop_period);
		
		/** @brief Does the actual initialization.
		  * @return Whether or not this communicator successfully initialized.
		  */
		virtual bool init();
		
		/** @brief Tells the Communicator that new controller outputs are available.
		  */
		virtual void transmitHook();
		
		/** @brief Called when the GUI signals a disable.
		  * Only needed for the actual
		  * robot -- the simulation doesn't need to respond to this.
		  */
		virtual void disable();
		
		/** @brief Called when the GUI wants to enable.
		  * The simulation doesn't need to react to this.
		  */
		virtual void enable();
		
		/** @brief Called to ESTOP the robot.
		  * The simulation doesn't need to react to this.
		  */
		virtual void eStop();
		
		/** @brief Take the robot out of eStop */
		virtual void leaveEStop();
};

#endif // COMMUNICATOR_H
