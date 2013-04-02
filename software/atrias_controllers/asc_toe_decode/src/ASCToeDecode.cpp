#include "asc_toe_decode/ASCToeDecode.hpp"

namespace atrias {
namespace controller {

ASCToeDecode::ASCToeDecode(AtriasController *parent, string name) :
	AtriasController(parent, name),
	log_out(this)
{
	// Initialize our gains
	this->log_out.data.filtered_val = 0.0;
 	this->filter_gain               = 0.05;
	this->threshold                 = 500.0;
	this->log_out.data.onGround     = false;
}

double ASCToeDecode::operator()(uint16_t force) {
	// Log our input data
	this->log_out.data.filter_gain = this->filter_gain;
	this->log_out.data.threshold   = this->threshold;
	this->log_out.data.force       = force;

	// Note: A higher raw force reading means a lower actual force.
	if (this->log_out.data.onGround && force > this->log_out.data.filtered_val + this->threshold) {
		this->log_out.data.onGround = false;
	} else if (!this->log_out.data.onGround && force < this->log_out.data.filtered_val - this->threshold) {
		this->log_out.data.onGround = true;
	}

	this->log_out.data.filtered_val += this->filter_gain * (force - this->log_out.data.filtered_val);

	// Transmit the log data
	this->log_out.send();

	// Return our output command -- sending the log data does not change
	// this value.
	return this->log_out.data.onGround;
}

bool ASCToeDecode::onGround() {
	return this->log_out.data.onGround;
}

}
}

// vim: noexpandtab
