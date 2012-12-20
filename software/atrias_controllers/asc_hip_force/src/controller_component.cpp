/*! \file controller_component.cpp
 *  \author Ryan
 *  \brief Orocos Component code for the asc_hip_force subcontroller.
 */

#include <asc_hip_force/controller_component.h>

namespace atrias {
namespace controller {

ASCHipForce::ASCHipForce(std::string name) :
	RTT::TaskContext(name),
	flightP(150.0),
	flightD(10.0),
	stanceP(0.0),
	stanceD(10.0),
	toeFilterGain(0.05),
	toeThreshold(500.0)
{
	this->provides("hipForce")
	->addOperation("runController", &ASCHipForce::runController, this, ClientThread)
	.doc("Run the controller.");
	this->provides("hipForce")->addOperation("getOnGround", &ASCHipForce::getOnGround, this, ClientThread)
	.doc("Returns whether or not this leg is touching the ground. Must be called after runController()");

	this->addProperty("flightP", flightP).doc("Flight P gain.");
	this->addProperty("flightD", flightD).doc("Flight D gain.");
	this->addProperty("stanceP", stanceP).doc("Stance P gain.");
	this->addProperty("stanceD", stanceD).doc("Stance D gain.");
	this->addProperty("toeFilterGain", toeFilterGain).doc("Toe decoder's filter gain.");
	this->addProperty("toeThreshold",  toeThreshold ).doc("Toe decoder's detection threshold.");

	log(Info) << "[ASCHipForce] Constructed!" << endlog();
}

// Put control code here.
double ASCHipForce::runController(uint16_t toeSwitch, int32_t kneeForce, double legBodyAngle, double legBodyVelocity) {
	// Set the gains/thresholds for the toe decoder.
	toeFilterGainProperty.set(toeFilterGain);
	toeThresholdProperty.set(toeThreshold);

	// Determine if we're in flight or stance.
	onGround = runToeDecode(toeSwitch);

	// Act differently depending on if we're in flight or stance.
	if (onGround) {
		P.set(stanceP);
		D.set(stanceD);
		return runPD(0.0, kneeForce, 0.0, legBodyVelocity);
	} else {
		P.set(flightP);
		D.set(flightD);
		return runPD(3.0 * M_PI / 2.0, legBodyAngle, 0.0, legBodyVelocity);
	}
}

bool ASCHipForce::getOnGround() {
	return onGround;
}

bool ASCHipForce::configureHook() {
	RTT::TaskContext* toeDecodeInstance = toeLoader.load(this, "asc_toe_decode", "ASCToeDecode");
	if (!toeDecodeInstance)
		return false;
	runToeDecode = toeDecodeInstance->provides("toeDecode")->getOperation("runController");
	toeFilterGainProperty = toeDecodeInstance->properties()->getProperty("filterGain");
	toeThresholdProperty  = toeDecodeInstance->properties()->getProperty("threshold");

	RTT::TaskContext* pdInstance = pdLoader.load(this, "asc_pd", "ASCPD");
	if (!pdInstance)
		return false;
	runPD = pdInstance->provides("pd")->getOperation("runController");
	P = pdInstance->properties()->getProperty("P");
	D = pdInstance->properties()->getProperty("D");

	if (!runToeDecode.ready()          ||
	    !toeFilterGainProperty.ready() ||
	    !toeThresholdProperty.ready()  ||
	    !runPD.ready()                 ||
	    !P.ready()                     ||
	    !D.ready()) {
		// Something went wrong.
		return false;
	}

	log(Info) << "[ASCHipForce] configured!" << endlog();
	return true;
}

bool ASCHipForce::startHook() {
	log(Info) << "[ASCHipForce] started!" << endlog();
	return true;
}

void ASCHipForce::updateHook() {
}

void ASCHipForce::stopHook() {
	log(Info) << "[ASCHipForce] stopped!" << endlog();
}

void ASCHipForce::cleanupHook() {
	log(Info) << "[ASCHipForce] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCHipForce)

}
}

// vim: noexpandtab
