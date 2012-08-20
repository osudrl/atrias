#ifndef ECATSLAVESMANAGER_H
#define ECATSLAVESMANAGER_H

/** @file
  * @brief Describes the interface for an ECat slaves manager -- whether for the
  * EtherCAT slave card or for the MedullaManager.
  */

// Orocos
#include <rtt/os/TimeService.hpp>

class ECatSlavesManager {
	public:
		/** @brief Tells the ECatSlavesManager that new data -- ready for
		  * processing -- is available.
		  */
		virtual void processReceiveData(RTT::os::TimeService::nsecs deltaTime);
		virtual void processReceiveData();
		
		/** @brief Tells the ECatSlavesManager to read in the new controller
		  * outputs.
		  */
		virtual void processTransmitData();
		
		/** @brief Disable/enable the robot
		  */
		virtual void setEnabled(bool enabled);
		
		/** @brief Tell the ECatSlavesManager to ESTOP (may be ignored by sim).
		  */
		virtual void eStop();
		
		/** @brief Take the robot out of EStop.
		  */
		virtual void leaveEStop();
		
		/** @brief Returns whether or not the ESTOP is active.
		  * @return Whether or not the robot it EStopped.
		  */
		virtual bool getEStopped();
};

#endif // ECATSLAVESMANAGER_H
