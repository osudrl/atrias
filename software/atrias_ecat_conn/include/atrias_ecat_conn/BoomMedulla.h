#ifndef BOOMMEDULLA_H
#define BOOMMEDULLA_H

#include <stdint.h>

#include "atrias_ecat_conn/Medulla.h"

namespace atrias {

namespace ecatConn {

class BoomMedulla : public Medulla {
	// Stuff sent to the medulla
	uint8_t*  command;
	uint16_t* counter;
	
	// Stuff received from the medulla
	uint8_t*  id;
	uint8_t*  state;
	uint8_t*  timingCounter;
	uint8_t*  errorFlags;
	
	uint32_t* xEncoder;
	uint16_t* xTimestamp;
	
	uint32_t* pitchEncoder;
	uint16_t* pitchTimestamp;
	
	uint32_t* zEncoder;
	uint16_t* zTimestamp;
	
	uint16_t* logicVoltage;
	
	public:
		/** @brief Does SOEM's slave-specific init.
		  * @param inputs A pointer to this slave's inputs.
		  * @param outputs A pointer to this slave's outputs.
		  */
		BoomMedulla(uint8_t* inputs, uint8_t* outputs);
};

}

}

#endif // BOOMMEDULLA_H
