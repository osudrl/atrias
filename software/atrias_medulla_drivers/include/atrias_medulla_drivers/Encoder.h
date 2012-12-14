#ifndef ENCODER_H
#define ENCODER_H

/** @file Encoder.h
  * @brief This class decodes an encoder reading.
  */

// Standard libs
#include <cstdint>

namespace atrias {

namespace medullaDrivers {

class Encoder {
	public:
		/** @brief Get this encoder's current position.
		  * @return The position of this encoder.
		  */
		double getPos();

		/** @brief Get this encoder's current velocity.
		  * @param deltaTime The time between the last \a update() and the one before it.
		  * @return This encoder's current velocity.
		  */
		double getVel(double deltaTime);

		/** @brief This initializes this Encoder.
		  * @param bits         The number of "bits" in this encoder's output (for wraparound compensation).
		  * @param calibReading This encoder's reading at its calibration position.
		  * @param calibLoc     This encoder's calibration position.
		  * @param scaling      The units of output change per encoder tick
		  */
		void init(int bits, uint32_t calibReading, double calibLoc, double scaling);

		/** @brief Updates this encoder's position and velocity
		  * @param reading This encoder's current reading
		  * This must be run each cycle before getPos and getVel
		  */
		void update(uint32_t reading);
	
	private:
		/** @brief This is the calibration position.
		  */
		double calibPos;

		/** @brief This stores the reading at the calibration position.
		  */
		uint32_t calibValue;

		/** @brief Whether or not we've calibrated curPos using calibValue.
		  */
		bool calibrated;

		/** @brief This is this encoder's current position.
		  * This is relative to the calibration location.
		  */
		intmax_t curPos;

		/** @brief This is the change in position between the last two calls of \a update()
		  * Used for velocity calculation.
		  */
		int32_t deltaPos;

		/** @brief This encoder's last reading. Used to calculate deltaPos
		  */
		uint32_t lastReading;

		/** @brief This stores the encoder's number of bits.
		  * Used for wraparound compensation
		  */
		int numBits;

		/** @brief This stores the units of output change per encoder tick
		  * Used to scale positions and velocities.
		  */
		double scalingRatio;

		/** @brief This is an integer modulo function.
		  * @param a The dividend
		  * @param b The divisor
		  * @return a modulo b
		  */
		intmax_t mod(intmax_t a, intmax_t b);
};

}

}

#endif // ENCODER_H

// vim: noexpandtab
