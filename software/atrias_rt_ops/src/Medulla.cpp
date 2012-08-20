#include "atrias_rt_ops/Medulla.h"

Medulla::Medulla() {
	local_counter = 0;
	state_command = medulla_state_idle;
}

void Medulla::setCmdState(medulla_state_t new_cmd_state) {
	state_command = new_cmd_state;
}

double Medulla::processADCValue(uint16_t adc_value) {
	// TODO: fix the offset here...
	return ((double) adc_value) * (MEDULLA_ADC_MAX_VOLTS/(4095.0));
}

double Medulla::processThermistorValue(uint16_t adc_value) {
	// Fixme! This isn't right at all...
	return 4700.0/((3.26/processADCValue(adc_value)) - 1.0);
}

double Medulla::processAmplifierCurrent(int16_t value) {
	return ((double) value) * 60.0 / 8192;
}

template <class T>
void Medulla::setPdoPointer (uint8_t* &cur_index, T* &pdo_pointer) {
	pdo_pointer = (T*) cur_index;
	cur_index  += sizeof(T);
}

// Let's force creation of the different template instance types we need.
template void Medulla::setPdoPointer<uint8_t>  (uint8_t* &cur_index, uint8_t*  &pdo_pointer);
template void Medulla::setPdoPointer<uint16_t> (uint8_t* &cur_index, uint16_t* &pdo_pointer);
template void Medulla::setPdoPointer<int32_t>  (uint8_t* &cur_index, int32_t*  &pdo_pointer);
template void Medulla::setPdoPointer<uint32_t> (uint8_t* &cur_index, uint32_t* &pdo_pointer);
template void Medulla::setPdoPointer<int16_t>  (uint8_t* &cur_index, int16_t*  &pdo_pointer);
