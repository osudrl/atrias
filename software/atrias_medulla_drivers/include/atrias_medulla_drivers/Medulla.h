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

/** @brief Contains data for a single PDO entry.
  * Used to point a Medulla driver to where a given PDO entry is.
  */
struct PDOEntryData {
	/** @brief The size of this PDO entry.
	  */
	int    size;
	
	/** @brief A pointer to the PDO entry pointer,
	  * so the Connector's code can set it to point at the right place.
	  */
	void** data;
};

/** @brief Contains all the data needed for the Connector to tell a Medulla
  * where its PDOs are.
  */
struct PDORegData {
	/** @brief The number of outputs for this Medulla
	  */
	int outputs;
	
	/** @brief The number of inputs for this Medulla.
	  */
	int inputs;
	
	/** @brief An array of PDOEntryData structs, for reading in PDO entry locations.
	  */
	PDOEntryData* pdoEntryDatas;
};


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
