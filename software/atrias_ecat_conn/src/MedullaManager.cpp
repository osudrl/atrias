#include "atrias_ecat_conn/MedullaManager.h"

namespace atrias {

namespace ecatConn {

MedullaManager::MedullaManager() {
	lLegA   = NULL;
	lLegB   = NULL;
	rLegA   = NULL;
	rLegB   = NULL;
	boom    = NULL;
	lLegHip = NULL;
	rLegHip = NULL;
}

void MedullaManager::slaveCardInit(ec_slavet slave) {
	uint8_t* inputs  = slave.inputs;
	uint8_t* outputs = slave.outputs;
	
	lLegA    = new LegMedulla(inputs, outputs);
	inputs  += lLegA->getInputsSize();
	outputs += lLegA->getOutputsSize();
	
	lLegB    = new LegMedulla(inputs, outputs);
	inputs  += lLegB->getInputsSize();
	outputs += lLegB->getOutputsSize();
	
	rLegA    = new LegMedulla(inputs, outputs);
	inputs  += rLegA->getInputsSize();
	outputs += rLegA->getOutputsSize();
	
	rLegB    = new LegMedulla(inputs, outputs);
	inputs  += rLegB->getInputsSize();
	outputs += rLegB->getOutputsSize();
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
				LegMedulla* medulla = new LegMedulla(slaves[i].inputs, slaves[i].outputs);
				log(RTT::Info) << "Leg medulla detected, ID: " << (int) medulla->getID() << RTT::endlog();
				
				if (medulla->getID() == MEDULLA_LEFT_LEG_A_ID) {
					log(RTT::Info) << "Left leg medulla A identified." << RTT::endlog();
					delete(lLegA);
					lLegA = medulla;
				} else if (medulla->getID() == MEDULLA_LEFT_LEG_B_ID) {
					log(RTT::Info) << "Left leg medulla B identified." << RTT::endlog();
					delete(lLegB);
					lLegB = medulla;
				} else if (medulla->getID() == MEDULLA_RIGHT_LEG_A_ID) {
					log(RTT::Info) << "Right leg medulla A identified." << RTT::endlog();
					delete(rLegA);
					rLegA = medulla;
				} else if (medulla->getID() == MEDULLA_RIGHT_LEG_B_ID) {
					log(RTT::Info) << "Right leg medulla B identified." << RTT::endlog();
					delete(rLegB);
					rLegB = medulla;
				} else {
					log(RTT::Warning) << "Leg medulla not identified." << RTT::endlog();
					delete(medulla);
				}
				
				break;
			}
			
			case MEDULLA_BOOM_PRODUCT_CODE: {
				delete(boom);
				boom = new BoomMedulla(slaves[i].inputs, slaves[i].outputs);
				log(RTT::Info) << "Boom medulla identified. ID: " <<
					(int) boom->getID() << RTT::endlog();
				
				break;
			}
			
			case MEDULLA_HIP_PRODUCT_CODE: {
				HipMedulla* medulla =
					new HipMedulla(slaves[i].inputs, slaves[i].outputs);
				log(RTT::Info) << "Hip medulla detected, ID: " <<
					(int) medulla->getID() << RTT::endlog();
				
				if (medulla->getID() == MEDULLA_LEFT_HIP_ID) {
					log(RTT::Info) << "Left hip medulla identified" <<
						RTT::endlog();
					delete(lLegHip);
					lLegHip = medulla;
				} else if (medulla->getID() == MEDULLA_RIGHT_HIP_ID) {
					log(RTT::Info) << "Left hip medulla identified" <<
						RTT::endlog();
					delete(lLegHip);
					lLegHip = medulla;
				} else {
					log(RTT::Warning) << "Hip medulla not identified."
						<< RTT::endlog();
				}
				
				break;
			}
			
			default: {
				log(RTT::Warning) <<
					"Unrecognized product code at 1-indexed position: "
					<< i << RTT::endlog();
			}
		}
	}
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
}

void MedullaManager::setTime(RTT::os::TimeService::nsecs time) {
	robotState.header.stamp.nsec = time % SECOND_IN_NANOSECONDS;
	robotState.header.stamp.sec  = (time - robotState.header.stamp.nsec) / SECOND_IN_NANOSECONDS;
}

atrias_msgs::robot_state MedullaManager::getRobotState() {
	return robotState;
}

}

}
