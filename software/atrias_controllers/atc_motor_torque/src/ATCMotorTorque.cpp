#include "atc_motor_torque/ATCMotorTorque.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCMotorTorque::ATCMotorTorque(string name) :
	ATC(name)
{
	// Nothing to see here.
}

void ATCMotorTorque::controller() {
	co.lLeg.motorCurrentA   = guiIn.des_motor_torque_left_A;
	co.lLeg.motorCurrentB   = guiIn.des_motor_torque_left_B;
	co.lLeg.motorCurrentHip = guiIn.des_motor_torque_left_hip;
	co.rLeg.motorCurrentA   = guiIn.des_motor_torque_right_A;
	co.rLeg.motorCurrentB   = guiIn.des_motor_torque_right_B;
	co.rLeg.motorCurrentHip = guiIn.des_motor_torque_right_hip;
}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCMotorTorque)

}
}

// vim: noexpandtab
