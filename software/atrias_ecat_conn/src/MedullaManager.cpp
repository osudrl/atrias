#include "atrias_ecat_conn/MedullaManager.h"

namespace atrias {

namespace ecatConn {

void MedullaManager::InputsConfig(uint8_t* slave_inputs, intptr_t* inputs_array, int num_entries) {
	for (int i = 0; i < num_entries; i++) {
		inputs_array[i] = (intptr_t) slave_inputs + i;
	}
}

void MedullaManager::OutputsConfig(uint8_t* slave_outputs, intptr_t* outputs_array, int num_entries) {
	for (int i = 0; i < num_entries; i++) {
		outputs_array[i] = (intptr_t) slave_outputs + i;
	}
}

MedullaManager::MedullaManager() {
	lLegA   = NULL;
	lLegB   = NULL;
	rLegA   = NULL;
	rLegB   = NULL;
	boom    = NULL;
	lLegHip = NULL;
	rLegHip = NULL;
}

MedullaManager::~MedullaManager() {
	delete(lLegA);
	delete(lLegB);
	delete(rLegA);
	delete(rLegB);
	delete(boom);
	delete(lLegHip);
	delete(rLegHip);
}

void MedullaManager::slaveCardInit(ec_slavet slave) {
	// Not implemented yet.
}

rtOps::RobotConfiguration MedullaManager::calcRobotConfiguration() {
	if (!lLegA || !lLegB) {
		return rtOps::RobotConfiguration::UNKNOWN;
	}
	
	// We have at least a left leg.
	
	if (!rLegA || !rLegB) {
		// We are a monopod
		if (lLegHip)
			return rtOps::RobotConfiguration::LEFT_LEG_HIP;
		else
			return rtOps::RobotConfiguration::LEFT_LEG_NOHIP;
	}
	
	// We are a biped
	
	if (!lLegHip || !rLegHip)
		return rtOps::RobotConfiguration::BIPED_NOHIP;
	else
		return rtOps::RobotConfiguration::BIPED_FULL;
}

void MedullaManager::initBoomMedulla(ec_slavet slave) {
	delete(boom);
	boom = new medullaDrivers::BoomMedulla();
	fillInPDORegData(boom->getPDORegData(), (uint8_t*) slave.outputs, (uint8_t*) slave.inputs);
	boom->postOpInit();
	log(RTT::Info) << "Boom medulla identified. ID: " <<
		(int) boom->getID() << RTT::endlog();
}

void MedullaManager::initHipMedulla(ec_slavet slave) {
	medullaDrivers::HipMedulla* medulla =
		new medullaDrivers::HipMedulla();
	fillInPDORegData(medulla->getPDORegData(), (uint8_t*) slave.outputs, (uint8_t*) slave.inputs);
	medulla->postOpInit();
	log(RTT::Info) << "Hip medulla detected, ID: " <<
		(int) medulla->getID() << RTT::endlog();
	
	switch (medulla->getID()) {
		case MEDULLA_LEFT_HIP_ID:
			log(RTT::Info) << "Left hip medulla identified" <<
				RTT::endlog();
			delete(lLegHip);
			lLegHip = medulla;
			break;
		case MEDULLA_RIGHT_HIP_ID:
			log(RTT::Info) << "Right hip medulla identified" <<
				RTT::endlog();
			delete(rLegHip);
			rLegHip = medulla;
			break;
		default:
			log(RTT::Warning) << "Hip medulla not identified."
				<< RTT::endlog();
			delete(medulla);
			break;
	}
}

void MedullaManager::initLegMedulla(ec_slavet slave) {
	medullaDrivers::LegMedulla* medulla =
		new medullaDrivers::LegMedulla();
	fillInPDORegData(medulla->getPDORegData(), (uint8_t*) slave.outputs, (uint8_t*) slave.inputs);
	medulla->postOpInit();
	log(RTT::Info) << "Leg medulla detected, ID: " <<
		(int) medulla->getID() << RTT::endlog();
	
	switch (medulla->getID()) {
		case MEDULLA_LEFT_LEG_A_ID:
			log(RTT::Info) << "Left leg medulla A identified." << RTT::endlog();
			delete(lLegA);
			lLegA = medulla;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			log(RTT::Info) << "Left leg medulla B identified." << RTT::endlog();
			delete(lLegB);
			lLegB = medulla;
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			log(RTT::Info) << "Right leg medulla A identified." << RTT::endlog();
			delete(rLegA);
			rLegA = medulla;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			log(RTT::Info) << "Right leg medulla B identified." << RTT::endlog();
			delete(rLegB);
			rLegB = medulla;
			break;
		default:
			log(RTT::Warning) << "Leg medulla not identified." << RTT::endlog();
			delete(medulla);
			break;
	}
}

void MedullaManager::fillInPDORegData(medullaDrivers::PDORegData pdo_reg_data,
                                      uint8_t* outputs, uint8_t* inputs) {
	uint8_t* cur_ptr = outputs;
	for (int i = 0; i < pdo_reg_data.outputs; i++) {
		*(pdo_reg_data.pdoEntryDatas[i].data) = cur_ptr;
		cur_ptr += pdo_reg_data.pdoEntryDatas[i].size;
	}
	
	cur_ptr = inputs;
	for (int i = pdo_reg_data.outputs; i < pdo_reg_data.outputs + pdo_reg_data.inputs; i++) {
		*(pdo_reg_data.pdoEntryDatas[i].data) = cur_ptr;
		cur_ptr += pdo_reg_data.pdoEntryDatas[i].size;
	}
}

void MedullaManager::medullasInit(ec_slavet slaves[], int slavecount) {
	// SOEM is 1-indexed.
	for (int i = 1; i <= slavecount; i++) {
		if (slaves[i].eep_man != MEDULLA_VENDOR_ID) {
			log(RTT::Warning) <<
				"Unrecognized product ID at 1-indexed position: "
				<< i << RTT::endlog();
			continue;
		}
		
		switch(slaves[i].eep_id) {
			case MEDULLA_LEG_PRODUCT_CODE: {
				initLegMedulla(slaves[i]);
				break;
			}
			
			case MEDULLA_BOOM_PRODUCT_CODE: {
				initBoomMedulla(slaves[i]);
				break;
			}
			
			case MEDULLA_HIP_PRODUCT_CODE: {
				initHipMedulla(slaves[i]);
				break;
			}
			
			default: {
				log(RTT::Warning) <<
					"Unrecognized product code at 1-indexed position: "
					<< i << RTT::endlog();
			}
		}
	}
	setRobotConfiguration(calcRobotConfiguration());
}

void MedullaManager::start(ec_slavet slaves[], int slavecount) {
	if (slaves[1].eep_id == KPA_SLAVE_CARD_PRODUCT_CODE) {
		log(RTT::Info) << "[ECatConn] Identified KPA slave card." << RTT::endlog();
		slaveCardInit(slaves[1]);
	} else {
		log(RTT::Info) << "[ECatConn] Did not identify slave card, configuring for Medulla-based operation." << RTT::endlog();
		medullasInit(slaves, slavecount);
	}
}

void MedullaManager::processReceiveData() {
	if (lLegA)
		lLegA->processReceiveData(robotState);
	if (lLegB)
		lLegB->processReceiveData(robotState);
	if (rLegA)
		rLegA->processReceiveData(robotState);
	if (rLegB)
		rLegB->processReceiveData(robotState);
	if (boom)
		boom->processReceiveData(robotState);
	if (lLegHip)
		lLegHip->processReceiveData(robotState);
	if (rLegHip)
		rLegHip->processReceiveData(robotState);
}

void MedullaManager::processTransmitData(atrias_msgs::controller_output& controller_output) {
	if (lLegA)
		lLegA->processTransmitData(controller_output);
	if (lLegB)
		lLegB->processTransmitData(controller_output);
	if (rLegA)
		rLegA->processTransmitData(controller_output);
	if (rLegB)
		rLegB->processTransmitData(controller_output);
	if (boom)
		boom->processTransmitData(controller_output);
	if (lLegHip)
		lLegHip->processTransmitData(controller_output);
	if (rLegHip)
		rLegHip->processTransmitData(controller_output);
}

void MedullaManager::setTime(RTT::os::TimeService::nsecs time, RTT::os::TimeService::nsecs receive_time) {
	robotState.header.stamp.nsec = time % SECOND_IN_NANOSECONDS;
	robotState.header.stamp.sec  = (time - robotState.header.stamp.nsec) / SECOND_IN_NANOSECONDS;
    robotState.receiveDCTime     = receive_time;
}

void MedullaManager::setTransmitTime(RTT::os::TimeService::nsecs transmit_time) {
    robotState.lastTransmitDCTime = transmit_time;
}

atrias_msgs::robot_state MedullaManager::getRobotState() {
	return robotState;
}

void MedullaManager::setRobotConfiguration(rtOps::RobotConfiguration new_robot_configuration) {
	robotState.robotConfiguration = (rtOps::RobotConfiguration_t) new_robot_configuration;
}

}

}

// vim: set noexpandtab:
