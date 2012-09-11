#ifndef MEDULLA_H
#define MEDULLA_H

/** @file
  * @brief Is the superclass for all specific medulla class types.
  */

#include <stdint.h>
#include <math.h>

#include "robot_invariant_defs.h"

namespace atrias {

namespace medullaDrivers {

class Medulla {
	protected:
		/** @brief Holds the counter value for feeding the master watchdog.
		  */
		uint16_t        local_counter;
			
		/** @brief Decodes a logic voltage.
		  * @param adc_value The voltage value from the ADC.
		  * @return The logic voltage for this Medulla
		  */
		double decodeLogicVoltage(uint16_t adc_value);
		
		/** @brief Decodes a motor voltage.
		  * @param adc_value The voltage value from the ADC.
		  * @return The motor voltage for this Medulla
		  */
		double decodeMotorVoltage(uint16_t adc_value);
		
		/** @brief  Processes an ADC value into a voltage.
		  * @param  adc_value The ADC value from the Medulla.
		  * @return The voltage this ADC is reading.
		  */
		double processADCValue(uint16_t adc_value);
		
		/** @brief Process an ADC value into a temperature (for thermistors).
		  * @param adc_value The ADC value reported by the Medulla
		  * @return The temperature of this thermistor.
		  */
		double processThermistorValue(uint16_t adc_value);
		
		/** @brief       Processes a raw reported amplifier current value into an amperage.
		  * @param value The raw reported amplifier current value.
		  * @return      The amperage this amplifier is outputting.
		  */
		double processAmplifierCurrent(int16_t value);
		
	public:
		/** @brief Does a bit of initialization.
		  */
		Medulla();
};

}

}

#endif // MEDULLA_H
