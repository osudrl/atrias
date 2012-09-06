#ifndef MEDULLA_H
#define MEDULLA_H

/** @file
  * @brief Is the superclass for all specific medulla class types.
  */

#include <stdint.h>
#include <math.h>

#include "robot_invariant_defs.h"

namespace atrias {

namespace ecatConn {

class Medulla {
	protected:
		/** @brief Holds the counter value for feeding the master watchdog.
		  */
		uint16_t        local_counter;
		
		/** @brief Sets a pointer to point to that pdo entry's location in
		  * RAM.
		  * @param pdo_pointer The pointer to be set to the PDO's location.
		  * @param cur_index A pointer holding the current reference address.
		  */
		template <class T>
			void setPdoPointer(uint8_t* &cur_index, T* &pdo_pointer);
			
		/** @brief Decodes a logic voltage.
		  * @param adc_value The voltage value from the ADC.
		  * @return The logic voltage for this Medulla
		  */
		double decodeLogicVoltage(uint16_t adc_value);
		
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
