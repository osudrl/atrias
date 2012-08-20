#include "atrias_rt_ops/MedullaManager.h"

MedullaManager::MedullaManager(RTOps* rt_ops, ec_slavet slaves[], int slavecount) {
	rtOps       = rt_ops;
	LlegA       = NULL;
	LlegB       = NULL;
	RlegA       = NULL;
	RlegB       = NULL;
	cmd_enabled = false;
	leave_estop = false;
	state_cmd   = medulla_state_idle;
	
	// SOEM is 1-indexed.
	for (int i = 1; i <= slavecount; i++) {
		if (slaves[i].eep_man != MEDULLA_VENDOR_ID)
			continue;
		
		switch(slaves[i].eep_id) {
			case MEDULLA_LEG_PRODUCT_CODE:
				LegMedulla* medulla = new LegMedulla(rtOps, &(slaves[i]));
				RTT::log(Info) << "Leg medulla detected, ID: " << (int) medulla->getID() << endlog();
				
				if (medulla->getID() == MEDULLA_LEFT_LEG_A_ID) {
					RTT::log(Info) << "Left leg medulla A identified." << endlog();
					delete(LlegA);
					LlegA = medulla;
				} else if (medulla->getID() == MEDULLA_LEFT_LEG_B_ID) {
					RTT::log(Info) << "Left leg medulla B identified." << endlog();
					delete(LlegB);
					LlegB = medulla;
				} else if (medulla->getID() == MEDULLA_RIGHT_LEG_A_ID) {
					RTT::log(Info) << "Right leg medulla A identified." << endlog();
					delete(RlegA);
					RlegA = medulla;
				} else if (medulla->getID() == MEDULLA_RIGHT_LEG_B_ID) {
					RTT::log(Info) << "Right leg medulla B identified." << endlog();
					delete(RlegB);
					RlegB = medulla;
				} else {
					RTT::log(Info) << "Leg medulla not identified." << endlog();
					delete(medulla);
				}
				
				break;
		}
	}
	processReceiveData();
}

void MedullaManager::processReceiveData() {
	if (LlegA)
		LlegA->processReceiveData();
	if (LlegB)
		LlegB->processReceiveData();
	if (RlegA)
		RlegA->processReceiveData();
	if (RlegB)
		RlegB->processReceiveData();
	return;
}

void MedullaManager::processTransmitData() {
	updateStateCmd();
	
	if (LlegA)
		LlegA->setCmdState(state_cmd);
	if (LlegB)
		LlegB->setCmdState(state_cmd);
	if (RlegA)
		RlegA->setCmdState(state_cmd);
	if (RlegB)
		RlegB->setCmdState(state_cmd);
		
	if (LlegA)
		LlegA->processTransmitData();
	if (LlegB)
		LlegB->processTransmitData();
	if (RlegA)
		RlegA->processTransmitData();
	if (RlegB)
		RlegB->processTransmitData();
}

void MedullaManager::updateStateCmd() {
	if (leave_estop) {
		state_cmd   = medulla_state_reset;
		leave_estop = false;
		return;
	}
	
	if (rtOps->controllerOutput.command == 2)
		eStop();
	
	if (state_cmd == medulla_state_error)
		return;
	
	if (shouldHalt()) {
		state_cmd = medulla_state_halt;
		return;
	}
	
	if (cmd_enabled && (rtOps->controllerOutput.command == 0)) {
		state_cmd = medulla_state_run;
		return;
	} else {
		state_cmd = medulla_state_idle;
		return;
	}
}

bool MedullaManager::shouldHalt() {
	// todo: put safety here.
	return false;
}

bool MedullaManager::getAllEStopped() {
	return true;
}

void MedullaManager::setEnabled(bool enabled) {
	cmd_enabled = enabled;
}

void MedullaManager::eStop() {
	state_cmd   = medulla_state_error;
	leave_estop = false;
}

void MedullaManager::leaveEStop() {
	// Don't leave EStop unless the GUI has commanded a disable.
	if (cmd_enabled)
		return;
	
	// Don't leave EStop unless all medullas are in error state.
	if (!getAllEStopped())
		return;
		
	leave_estop = true;
}
