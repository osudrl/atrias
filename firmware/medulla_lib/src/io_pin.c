#include "io_pin.h"

inline void io_set_direction (io_pin_t pin, io_pin_direction_t direction) {
	(*((uint8_t*)pin.io_port+direction)) = 1<<pin.pin;
	/* This works because adding the integer value of an io_pin_direction_t to
	the io_port pointer results in the address of either DIRSET or DIRCLR
	registers depending upon the desired direction*/
}

inline void io_set_output (io_pin_t pin, io_pin_level_t level) {
	(*((uint8_t*)pin.io_port+6+level)) = 1<<pin.pin;
}

inline void io_toggle_output(io_pin_t pin) {
	pin.io_port->OUTTGL = 1<<pin.pin;
}

inline io_pin_level_t io_get_input(io_pin_t pin) {
	if (pin.io_port->IN & 1<<pin.pin)
		return io_high;
	else
		return io_low; 
}

inline void io_pin_enable_pullup(io_pin_t pin) {
	*(pin.control_reg) |= PORT_OPC_PULLUP_gc;
}

inline void io_pin_disable_pullup(io_pin_t pin) {
	*(pin.control_reg) &= !PORT_OPC_PULLUP_gc;
}

