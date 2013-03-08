#include "atrias_medulla_drivers/Medulla.h"

namespace atrias {

namespace medullaDrivers {

Medulla::Medulla() {
	local_counter = 0;
}

double Medulla::decodeLogicVoltage(uint16_t adc_value) {
	return processADCValue(adc_value) * 6.0;
}

double Medulla::decodeMotorVoltage(uint16_t adc_value) {
	return (adc_value-MOTOR_VOLTAGE_C_OFFSET)*MOTOR_VOLTAGE_V_CAL/(MOTOR_VOLTAGE_C_CAL-MOTOR_VOLTAGE_C_OFFSET);
}

double Medulla::processADCValue(uint16_t adc_value) {
	return ((double) adc_value - MEDULLA_ADC_OFFSET_COUNTS) * (MEDULLA_ADC_MAX_VOLTS/(4095.0));
}

double Medulla::processThermistorValue(uint16_t adc_value) {
	// Whoa...
	// (Copied directly from the old ucontroller.h).
	return ((1.0/( (1.0/298.15) + (1.0/3988.0)*log(4700.0/((3.26/processADCValue(adc_value)) - 1.0)/10000))) - 273.15);
}

double Medulla::processAmplifierCurrent(int16_t value) {
	return ((double) value) * 60.0 / 8192;
}

}

}
