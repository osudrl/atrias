/**
  * @file ATC_SLIP_HOPPING.cpp
  * @author Mikhail Jones
  * @brief This implements a SLIP based template controller.
  */
  
#include "atc_slip_hopping/ATCSlipHopping.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCSlipHopping::ATCSlipHopping(string name) :
	ATC(name),
	ascSlipModel(this, "ascSlipModel")
{
	// Nothing to see here.
}

/* @brief This is the main function for the top-level controller.
 * @param rs The robot state is an inherited member.
 * @param co The controller output is an inhereted member.
 */
void ATCSlipHopping::controller() {
	
	/* Additionally, the following functions are available to command the robot state:
	 * commandHalt();    // Trigger a Halt
	 * commandEStop();   // Trigger an EStop
	 * commandDisable(); // Disable the robot
	 * setStartupEnabled(true/false) // Enable or disable the default startup controller
	 * setShutdownEnabled(true/false) // Enable or disable the shutdown controller
	 */
	 
    // Startup is handled by the ATC class.
    setStartupEnabled(true);

	// Controller state machine
	switch (controllerState) {
		// Standing
		case 1:
		
			break;
		
		// Running
		case 2:
		
			// SLIP running state machine
			switch (runningState) {
				// Stance phase
				case 1:
					
					break;
				// Flight phase	
				case 2:
				
					break;
					
			}
			break;
		
	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();
	 
}

// passiveStanceControl
void ATCSlipHopping::passiveStanceControl() {

	
}


// forceStanceControl
void ATCSlipHopping::forceStanceControl() {


}

// egbFlightControl
void ATCSlipHopping::egbFlightControl() {


}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipHopping)

}
}
