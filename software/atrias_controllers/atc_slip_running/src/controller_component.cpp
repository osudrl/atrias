/*! \file controller_component.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_slip_running controller.
 */

// Initialize ==================================================================
#include <atc_slip_running/controller_component.h>

namespace atrias {
namespace controller {

// ATCSlipRunning ================================================================
ATCSlipRunning::ATCSlipRunning(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in") {
    
    // Operations --------------------------------------------------------------
    // ATCSlipRunning
    this->provides("atc")->addOperation("runController",&ATCSlipRunning::runController, this, ClientThread).doc("Get robot_state from RTOps and return controller output.");
    
    // ASCCommonToolkit
	this->addProperty("ascCommonToolkit0Name", ascCommonToolkit0Name);

    // Variables ---------------------------------------------------------------

    // GUI ---------------------------------------------------------------------
    addEventPort(guiDataIn);
    addPort(guiDataOut);
    pubTimer = new GuiPublishTimer(20);

    // Logging -----------------------------------------------------------------
    // Create a port
    addPort(logPort);
    // Buffer port so we capture all data.
    ConnPolicy policy = RTT::ConnPolicy::buffer(100000);
    // Transport type = ROS
    policy.transport = 3;
    // ROS topic name
    policy.name_id = "/" + name + "_log";
    // Construct the stream between the port and ROS topic
    logPort.createStream(policy);
    log(Info) << "[ATCMT] Constructed!" << endlog();
    
}

// runController ===============================================================
atrias_msgs::controller_output ATCSlipRunning::runController(atrias_msgs::robot_state rs) {

    // Do nothing unless told otherwise
    co.lLeg.motorCurrentA = 0.0;
    co.lLeg.motorCurrentB = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA = 0.0;
    co.rLeg.motorCurrentB = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // Only run the controller when we're enabled
    if ((rtOps::RtOpsState)rs.rtOpsState != rtOps::RtOpsState::ENABLED)
        return co;

    // BEGIN CONTROL CODE ******************************************************

	// Current scaling state machine -------------------------------------------
	//if (guiIn.standing) {
	//	controllerState = 1;
	//} else if (guiIn.running) {
	//	controllerState = 2;
	//}
	
	
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

    // END CONTROL CODE ********************************************************

    // Command a run state
    co.command = medulla_state_run;

    // Let the GUI know the controller run state
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);
    
    // Send data to the GUI
    if (pubTimer->readyToSend()) guiDataOut.write(guiOut);

    // Stuff the msg and push to ROS for logging
    //logData.header = getROSHeader();
    //logPort.write(logData);

    // Output for RTOps
    return co;
    
} // ATCSlipRunning


// passiveStanceControl ===============================================================
void ATCSlipRunning::passiveStanceControl() {

}


// forceStanceControl ===============================================================
void ATCSlipRunning::forceStanceControl() {

}

// egbFlightControl ===============================================================
void ATCSlipRunning::egbFlightControl() {

	co.lLeg.motorCurrentA = 10.0;

}


// DON'T PUT CONTROL CODE BELOW HERE! ******************************************

// configureHook ===============================================================
bool ATCSlipRunning::configureHook() {
    // Log data stuff ----------------------------------------------------------
    //RTT::TaskContext* rtOpsPeer = this->getPeer("Deployer")->getPeer("atrias_rt");
    //if (rtOpsPeer) {
    //    getROSHeader = rtOpsPeer->provides("timestamps")->getOperation("getROSHeader");
    //} else {
    //    log(Warning) << "[ATCMT] Can't connect to the Deployer" << endlog();
    //}
    
    // Subcontrollers ----------------------------------------------------------
	// ASCCommonToolkit
	ascCommonToolkit0 = this->getPeer(ascCommonToolkit0Name);
	legStiffness0 = ascCommonToolkit0->provides("ascCommonToolkit")->getOperation("legStiffness");

	// Return ------------------------------------------------------------------
    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
    
} // configureHook

// startHook ===================================================================
bool ATCSlipRunning::startHook() {

    log(Info) << "[ATCMT] started!" << endlog();
    return true;
    
} // startHook

// updateHook ==================================================================
void ATCSlipRunning::updateHook() {

    guiDataIn.read(guiIn);
    
} // updateHook

// stopHook ====================================================================
void ATCSlipRunning::stopHook() {

    log(Info) << "[ATCMT] stopped!" << endlog();
    
} // stopHook

// cleanupHook =================================================================
void ATCSlipRunning::cleanupHook() {

    log(Info) << "[ATCMT] cleaned up!" << endlog();

} // cleanupHook

ORO_CREATE_COMPONENT(ATCSlipRunning)

} // namespace controller
} // namespace atrias
