#include "atrias_medulla_drivers/Encoder.h"

namespace atrias {

namespace medullaDrivers {

double Encoder::getPos() {
	return calibPos + scalingRatio * curPos;
}

double Encoder::getVel(double deltaTime) {
	return scalingRatio * deltaPos / deltaTime;
}

void Encoder::init(int bits, uint32_t calibReading, double calibLoc, double scaling) {
	numBits      = bits;
	calibValue   = calibReading;
	calibPos     = calibLoc;
	scalingRatio = scaling;
	calibrated   = false;
}

void Encoder::update(uint32_t reading) {
	intmax_t range     = 1 << numBits;
	intmax_t halfRange = range / 2;
	if (!calibrated) {
		curPos             = reading - calibValue;
		curPos             = mod(curPos + halfRange, range) - halfRange;
		lastReading        = reading;
		calibrated         = true;
	}

	deltaPos     = reading - lastReading;
	deltaPos     = mod(deltaPos + halfRange, range) - halfRange;
	curPos      += deltaPos;
	lastReading += deltaPos;
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
