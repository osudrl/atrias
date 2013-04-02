#include "asc_hip_force/ASCHipForce.hpp"

namespace atrias {
namespace controller {

ASCHipForce::ASCHipForce(AtriasController *parent, string name) :
	AtriasController(parent, name),
	flightPD(this, "flightPD"),
	stancePD(this, "stancePD"),
	toeDecode(this, "toeDecode")
{
	// Initialize our gains to something safe.
	this->flightP = 150.0;
	this->flightD = 10.0;

	this->stanceP = 0.0;
	this->stanceD = 10.0;

	// Copy over defaults from the toe decode controller
	this->toeFilterGain = this->toeDecode.filter_gain;
	this->toeThreshold  = this->toeDecode.threshold;
}

double ASCHipForce::operator()(const atrias_msgs::robot_state_leg &leg) {
	double output;

	// Set toe decoding gains.
	this->toeDecode.filter_gain = this->toeFilterGain;
	this->toeDecode.threshold   = this->toeThreshold;

	// The "&& false" temporarily disables stance phase control,
	// so the atc_hip_force controller can safely be run. (And the
	// toe decoding controller debugged).
	if (this->toeDecode(leg.toeSwitch) && false) {
		// We're on the ground, run stance phase control

		// Set stance gains
		this->stancePD.P = this->stanceP;
		this->stancePD.D = this->stanceD;

		// Run the controller
		output = this->stancePD(1.5 * M_PI, leg.hip.legBodyAngle, 0.0, leg.hip.legBodyVelocity);
	} else {
		// We're in flight, hold the leg vertical

		// Set flight controller gains
		this->flightPD.P = this->flightP;
		this->flightPD.D = this->flightD;

		// Run the controller
		// This is not a true PD controller on knee strain, but instead it sums a proportional
		// strain control term with a term proportional to the hip velocity (to act as a damper).
		output = this->flightPD(0.0, leg.kneeForce, 0.0, leg.hip.legBodyVelocity);
	}
	return output;
}

bool ASCHipForce::onGround() {
	return this->toeDecode.onGround();
}

}
}

// vim: noexpandtab
