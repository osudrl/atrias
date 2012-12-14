#include "atrias_medulla_drivers/Encoder.h"

namespace atrias {

namespace medullaDrivers {

double Encoder::getPos() {
	return calibPos + scalingRatio * curPos;
}

double Encoder::getVel() {
	return scalingRatio * deltaPos / deltaTime;
}

void Encoder::init(int bits, uint32_t calibReading, double calibLoc, double scaling) {
	numBits      = bits;
	calibValue   = calibReading;
	calibPos     = calibLoc;
	scalingRatio = scaling;
	calibrated   = false;
}

void Encoder::update(uint32_t reading, RTT::os::TimeService::nsecs delta_time, uint16_t timestamp) {
	intmax_t range     = 1 << numBits;
	intmax_t halfRange = range / 2;
	if (!calibrated) {
		curPos             = reading - calibValue;
		curPos             = mod(curPos + halfRange, range) - halfRange;
		lastReading        = reading;
		lastTimestamp      = timestamp;
		calibrated         = true;
	}

	deltaPos     = reading - lastReading;
	deltaPos     = mod(deltaPos + halfRange, range) - halfRange;
	curPos      += deltaPos;
	lastReading += deltaPos;

	deltaTime     = ((double) delta_time) / SECOND_IN_NANOSECONDS;
	deltaTime    += ((int16_t) (timestamp - lastTimestamp)) / MEDULLA_TIMER_FREQ;
	lastTimestamp = timestamp;
}

intmax_t Encoder::mod(intmax_t a, intmax_t b) {
	intmax_t out = a % b;
	out         += b;
	out          = out % b;
	return out;
}

}

}

// vim: noexpandtab
