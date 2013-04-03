/**
  * @file ATC_SLIP_RUNNING.cpp
  * @author Mikhail Jones
  * @brief This implements a SLIP based template controller.
  */
  
#include "atc_slip_running/ATCSlipRunning.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCSlipRunning::ATCSlipRunning(string name) :
	ATC(name),
	commonToolkit(this, "commonToolkit")
{
	// Nothing to see here.
}

/* @brief This is the main function for the top-level controller.
 * @param rs The robot state is an inherited member.
 * @param co The controller output is an inhereted member.
 */
void ATCSlipRunning::controller() {
	
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
	
	//Stand
	//Run
		//Right leg stance
			//Right leg force control
			//Left leg mirror angle
		//Left leg flight
			//Until appex
				//Left leg smoothly moves from current pos to appex egb angle
				//Right leg mirrors angle and shortens 15%
			//Until touchdown
				//Left leg tracks egb angle
				//Right leg mirrors angle and shortens 15%
		//Left leg stance
			
		//Right leg flight
	 
}

// passiveStanceControl
void ATCSlipRunning::passiveStanceControl() {

	k = commonToolkit.legStiffness(0.85, 0.85);
	
}


// forceStanceControl
void ATCSlipRunning::forceStanceControl() {



}

// egbFlightControl
void ATCSlipRunning::egbFlightControl() {

	co.lLeg.motorCurrentA = 0.0;

}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipRunning)

}
}
