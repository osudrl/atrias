#include "atrias_ecat_conn/BoomMedulla.h"

namespace atrias {

namespace ecatConn {

BoomMedulla::BoomMedulla(uint8_t* inputs, uint8_t* outputs) : Medulla() {
	uint8_t* cur_index = outputs;
	
	setPdoPointer(cur_index, command);
	setPdoPointer(cur_index, counter);
	
	cur_index = inputs;
	
	setPdoPointer(cur_index, id);
	setPdoPointer(cur_index, state);
	setPdoPointer(cur_index, timingCounter);
	setPdoPointer(cur_index, errorFlags);
	
	setPdoPointer(cur_index, xEncoder);
	setPdoPointer(cur_index, xTimestamp);
	
	setPdoPointer(cur_index, pitchEncoder);
	setPdoPointer(cur_index, pitchTimestamp);
	
	setPdoPointer(cur_index, zEncoder);
	setPdoPointer(cur_index, zTimestamp);
	
	setPdoPointer(cur_index, logicVoltage);
}

}

}
