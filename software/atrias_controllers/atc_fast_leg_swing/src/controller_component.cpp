/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_fast_leg_swing controller.
 */

#include <atc_fast_leg_swing/controller_component.h>

namespace atrias {
namespace controller {

ATCFastLegSwing::ATCFastLegSwing(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log"),
	guiDataIn("gui_data_in")
{
	this->provides("atc")
	->addOperation("runController", &ATCFastLegSwing::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	// Add properties
    this->addProperty("spg0Name", spg0Name);
    this->addProperty("spg1Name", spg1Name);
    this->addProperty("spg2Name", spg2Name);
    this->addProperty("spg3Name", spg3Name);
    this->addProperty("spg4Name", spg4Name);
    this->addProperty("spg5Name", spg5Name);
	this->addProperty("pathGenerator0Name", pathGenerator0Name);
	this->addProperty("pathGenerator1Name", pathGenerator1Name);
	this->addProperty("pathGenerator2Name", pathGenerator2Name);
	this->addProperty("pathGenerator3Name", pathGenerator3Name);
	this->addProperty("pathGenerator4Name", pathGenerator4Name);
	this->addProperty("pathGenerator5Name", pathGenerator5Name);
	this->addProperty("pd0Name", pd0Name);
	this->addProperty("pd1Name", pd1Name);
	this->addProperty("pd2Name", pd2Name);
	this->addProperty("pd3Name", pd3Name);
	this->addProperty("pd4Name", pd4Name);
	this->addProperty("pd5Name", pd5Name);
	
	// For the GUI
	addEventPort(guiDataIn);
	pubTimer = new GuiPublishTimer(20);
	
	lHipStart = 3.0 * M_PI / 2.0;
	rHipStart = 3.0 * M_PI / 2.0;

    demoRunning = false;   // Have the smooth path generators been initialized?

	// Logging
	// Create a port
	addPort(logPort);
	// Unbuffered
	ConnPolicy policy = RTT::ConnPolicy::data();
	// Transport type = ROS
	policy.transport = 3;
	// ROS topic name
	policy.name_id = "/" + name + "_log";
	// Construct the stream between the port and ROS topic
	logPort.createStream(policy);

	co.command = medulla_state_run;

	log(Info) << "[ATCFLS] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCFastLegSwing::runController(atrias_msgs::robot_state rs) {
    // Whether we should extend and retract, as opposed to swinging the legs.
    bool   extend  = guiIn.mode;
    double freq    = guiIn.frequency;
    double legampl = guiIn.leg_magnitude;
    double hipampl = guiIn.hip_magnitude;
    double legP    = guiIn.leg_p_gain;
    double legD    = guiIn.leg_d_gain;
    double hipP    = guiIn.hip_p_gain;
    double hipD    = guiIn.hip_d_gain;

    MotorState desiredLAState,
               desiredLBState,
               desiredLHState,
               desiredRAState,
               desiredRBState,
               desiredRHState;

    // Things to do when the controller is disabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE) {
        // Calculate neutral (powered) hip angle.
        lHipStart = rs.lLeg.hip.legBodyAngle + .05;
        rHipStart = rs.rLeg.hip.legBodyAngle - .05;

        // Keep desired motor angles equal to the current motor angles so the
        // motors don't jump when the controller is enabled.
        desiredLAState.ang = rs.lLeg.halfA.motorAngle;
        desiredLBState.ang = rs.lLeg.halfB.motorAngle;
        desiredLHState.ang = rs.lLeg.hip.legBodyAngle;
        desiredRAState.ang = rs.rLeg.halfA.motorAngle;
        desiredRBState.ang = rs.rLeg.halfB.motorAngle;
        desiredRHState.ang = rs.rLeg.hip.legBodyAngle;

        // Reset smooth path generators. This is needed in case the user
        // disables the controller before disabling the demo, which is a
        // problem since the generators do not run when the controller is
        // disabled, which means they will finish only once the controller is
        // re-enabled, and that only with outdated data.
        spg0IsFinished = true;
        spg1IsFinished = true;
        spg2IsFinished = true;
        spg3IsFinished = true;
        spg4IsFinished = true;
        spg5IsFinished = true;
    }

    // GUI says disable demo but demo is still running.
    if (!guiIn.demoEnabled && demoRunning) {
        if (extend) {
        	spg0Init(rs.lLeg.halfA.motorAngle, 0.851598663,      2.0);
        	spg1Init(rs.lLeg.halfB.motorAngle, 2.289994011,  2.0);
        	spg2Init(rs.lLeg.hip.legBodyAngle, lHipStart, 2.0);
        	spg3Init(rs.rLeg.halfA.motorAngle, 0.851598663,      2.0);
        	spg4Init(rs.rLeg.halfB.motorAngle, 2.289994011,  2.0);
        	spg5Init(rs.rLeg.hip.legBodyAngle, rHipStart, 2.0);
        }
        else {
        	spg0Init(rs.lLeg.halfA.motorAngle, M_PI/3.0,      2.0);
        	spg1Init(rs.lLeg.halfB.motorAngle, 2.0*M_PI/3.0,  2.0);
        	spg2Init(rs.lLeg.hip.legBodyAngle, lHipStart, 2.0);
        	spg3Init(rs.rLeg.halfA.motorAngle, M_PI/3.0,      2.0);
        	spg4Init(rs.rLeg.halfB.motorAngle, 2.0*M_PI/3.0,  2.0);
        	spg5Init(rs.rLeg.hip.legBodyAngle, rHipStart, 2.0);
        }

        demoRunning = false;
        log(Info) << "[ATCFLS] Demo disabled." << endlog();
    }

    // GUI says enable demo but demo is not running.
    else if (guiIn.demoEnabled && !demoRunning) {
        if (extend) {
        	spg0Init(rs.lLeg.halfA.motorAngle, 0.851598663,      2.0);
        	spg1Init(rs.lLeg.halfB.motorAngle, 2.289994011,  2.0);
        	spg2Init(rs.lLeg.hip.legBodyAngle, lHipStart, 2.0);
        	spg3Init(rs.rLeg.halfA.motorAngle, 0.851598663,      2.0);
        	spg4Init(rs.rLeg.halfB.motorAngle, 2.289994011,  2.0);
        	spg5Init(rs.rLeg.hip.legBodyAngle, rHipStart, 2.0);
        }
        else {
        	spg0Init(rs.lLeg.halfA.motorAngle, M_PI/3.0,      2.0);
        	spg1Init(rs.lLeg.halfB.motorAngle, 2.0*M_PI/3.0,  2.0);
        	spg2Init(rs.lLeg.hip.legBodyAngle, lHipStart, 2.0);
        	spg3Init(rs.rLeg.halfA.motorAngle, M_PI/3.0,      2.0);
        	spg4Init(rs.rLeg.halfB.motorAngle, 2.0*M_PI/3.0,  2.0);
        	spg5Init(rs.rLeg.hip.legBodyAngle, rHipStart, 2.0);
        }

        demoRunning = true;
        log(Info) << "[ATCFLS] Demo enabled." << endlog();
    }

    // Run Ryan's code if demo is running and initialization has finished.
    if (demoRunning &&
            spg0IsFinished &&
            spg1IsFinished &&
            spg2IsFinished &&
            spg3IsFinished &&
            spg4IsFinished &&
            spg5IsFinished) {

        path1ControllerSetPhase((extend) ? 1.0 : 0.0);
        path2ControllerSetPhase((extend) ? 1.0 : 0.5);
        path3ControllerSetPhase((extend) ? 1.0 : 1.0);
        path4ControllerSetPhase((extend) ? 0.0 : 1.0);
        path5ControllerSetPhase((extend) ? 1.0 : 1.5);

        desiredLAState = path0Controller(freq, legampl);
	  if (extend)
	        desiredLAState.ang += 0.851598663;
	  else
	        desiredLAState.ang += M_PI / 3.0;

        desiredLBState = path1Controller(freq, legampl);
	  if	(extend)
        	desiredLBState.ang += 2.289994011;
	  else
        	desiredLBState.ang += 2.0 * M_PI / 3.0;

        desiredLHState = path2Controller((extend) ? freq : (2.0*freq), hipampl);
        desiredLHState.ang += hipampl + lHipStart;

        desiredRAState = path3Controller(freq, legampl);
	  if (extend)
        	desiredRAState.ang += 0.851598663;
	  else
        	desiredRAState.ang += M_PI / 3.0;

        desiredRBState = path4Controller(freq, legampl);
	  if (extend) 
	        desiredRBState.ang += 2.289994011;
	  else
        	desiredRBState.ang += 2.0 * M_PI / 3.0;

        desiredRHState = path5Controller((extend) ? freq : (2.0*freq), hipampl);
        desiredRHState.ang += rHipStart - hipampl;

        double vertical = 3.0*M_PI/2.0;
        double inAngle  = M_PI/180.0*10.0;
        double outAngle = M_PI/180.0*20.0;
        if (desiredLHState.ang < (vertical - inAngle))
            desiredLHState.ang =  vertical - inAngle;
        if (desiredLHState.ang > (vertical + outAngle))
            desiredLHState.ang =  vertical + outAngle;
        if (desiredRHState.ang > (vertical + inAngle))
            desiredRHState.ang =  vertical + inAngle;
        if (desiredRHState.ang < (vertical - outAngle))
            desiredRHState.ang =  vertical - outAngle;
    }

    // Otherwise, smoothly move to either the started or stopped position.
    else {
        path0ControllerReset();
        path1ControllerReset();
        path2ControllerReset();
        path3ControllerReset();
        path4ControllerReset();
        path5ControllerReset();

        // Run smooth path generators only when needed (especially only if the
        // controller is enabled). Otherwise, they will put desired motor
        // angles at positions that may cause the controller to jump upon
        // enable.
        if ((uint8_t)rs.cmState == (uint8_t)controllerManager::RtOpsCommand::ENABLE &&
               (!spg0IsFinished ||
                !spg1IsFinished ||
                !spg2IsFinished ||
                !spg3IsFinished ||
                !spg4IsFinished ||
                !spg5IsFinished)) {
            desiredLAState = spg0RunController();
            desiredLBState = spg1RunController();
            desiredLHState = spg2RunController();
            desiredRAState = spg3RunController();
            desiredRBState = spg4RunController();
            desiredRHState = spg5RunController();
        }
    }

	P0.set(legP);
	D0.set(legD);
	P1.set(legP);
	D1.set(legD);
	P2.set(hipP);
	D2.set(hipD);
	P3.set(legP);
	D3.set(legD);
	P4.set(legP);
	D4.set(legD);
	P5.set(hipP);
	D5.set(hipD);
	
	co.lLeg.motorCurrentA   = pd0Controller(desiredLAState.ang, rs.lLeg.halfA.motorAngle, desiredLAState.vel, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB   = pd1Controller(desiredLBState.ang, rs.lLeg.halfB.motorAngle, desiredLBState.vel, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = pd2Controller(desiredLHState.ang, rs.lLeg.hip.legBodyAngle, desiredLHState.vel, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentA   = pd3Controller(desiredRAState.ang, rs.rLeg.halfA.motorAngle, desiredRAState.vel, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB   = pd4Controller(desiredRBState.ang, rs.rLeg.halfB.motorAngle, desiredRBState.vel, rs.rLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentHip = pd5Controller(desiredRHState.ang, rs.rLeg.hip.legBodyAngle, desiredRHState.vel, rs.rLeg.hip.legBodyVelocity);
	
	// Stuff the msg and push to ROS for logging
	logData.desiredState = 0.0;
	logPort.write(logData);

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCFastLegSwing::configureHook() {
	// Connect to the subcontrollers
	// Get references to subcontroller component properties
    spg0 = this->getPeer(spg0Name);
    if (spg0) {
        spg0Init          = spg0->provides("smoothPath")->getOperation("init");
        spg0RunController = spg0->provides("smoothPath")->getOperation("runController");
        spg0IsFinished    = spg0->properties()->getProperty("isFinished");
    }

    spg1 = this->getPeer(spg1Name);
    if (spg1) {
        spg1Init          = spg1->provides("smoothPath")->getOperation("init");
        spg1RunController = spg1->provides("smoothPath")->getOperation("runController");
        spg1IsFinished    = spg1->properties()->getProperty("isFinished");
    }

    spg2 = this->getPeer(spg2Name);
    if (spg2) {
        spg2Init          = spg2->provides("smoothPath")->getOperation("init");
        spg2RunController = spg2->provides("smoothPath")->getOperation("runController");
        spg2IsFinished    = spg2->properties()->getProperty("isFinished");
    }

    spg3 = this->getPeer(spg3Name);
    if (spg3) {
        spg3Init          = spg3->provides("smoothPath")->getOperation("init");
        spg3RunController = spg3->provides("smoothPath")->getOperation("runController");
        spg3IsFinished    = spg3->properties()->getProperty("isFinished");
    }

    spg4 = this->getPeer(spg4Name);
    if (spg4) {
        spg4Init          = spg4->provides("smoothPath")->getOperation("init");
        spg4RunController = spg4->provides("smoothPath")->getOperation("runController");
        spg4IsFinished    = spg4->properties()->getProperty("isFinished");
    }

    spg5 = this->getPeer(spg5Name);
    if (spg5) {
        spg5Init          = spg5->provides("smoothPath")->getOperation("init");
        spg5RunController = spg5->provides("smoothPath")->getOperation("runController");
        spg5IsFinished    = spg5->properties()->getProperty("isFinished");
    }

	pathGenerator0 = this->getPeer(pathGenerator0Name);
	if (pathGenerator0) {
		path0Controller      = pathGenerator0->provides("parabolaGen")->getOperation("runController");
		path0ControllerReset = pathGenerator0->provides("parabolaGen")->getOperation("reset");
	}
		
	pathGenerator1 = this->getPeer(pathGenerator1Name);
	if (pathGenerator1) {
		path1Controller         = pathGenerator1->provides("parabolaGen")->getOperation("runController");
		path1ControllerSetPhase = pathGenerator1->provides("parabolaGen")->getOperation("setPhase");
		path1ControllerReset    = pathGenerator1->provides("parabolaGen")->getOperation("reset");
	}
	
	pathGenerator2 = this->getPeer(pathGenerator2Name);
	if (pathGenerator2) {
		path2Controller         = pathGenerator2->provides("parabolaGen")->getOperation("runController");
		path2ControllerSetPhase = pathGenerator2->provides("parabolaGen")->getOperation("setPhase");
		path2ControllerReset    = pathGenerator2->provides("parabolaGen")->getOperation("reset");
	}
	
	pathGenerator3 = this->getPeer(pathGenerator3Name);
	if (pathGenerator3) {
		path3Controller         = pathGenerator3->provides("parabolaGen")->getOperation("runController");
		path3ControllerSetPhase = pathGenerator3->provides("parabolaGen")->getOperation("setPhase");
		path3ControllerReset    = pathGenerator3->provides("parabolaGen")->getOperation("reset");
	}
		
	pathGenerator4 = this->getPeer(pathGenerator4Name);
	if (pathGenerator4) {
		path4Controller         = pathGenerator4->provides("parabolaGen")->getOperation("runController");
		path4ControllerSetPhase = pathGenerator4->provides("parabolaGen")->getOperation("setPhase");
		path4ControllerReset    = pathGenerator4->provides("parabolaGen")->getOperation("reset");
	}
	
	pathGenerator5 = this->getPeer(pathGenerator5Name);
	if (pathGenerator5) {
		path5Controller         = pathGenerator5->provides("parabolaGen")->getOperation("runController");
		path5ControllerSetPhase = pathGenerator5->provides("parabolaGen")->getOperation("setPhase");
		path5ControllerReset    = pathGenerator5->provides("parabolaGen")->getOperation("reset");
	}
	
	pd0 = this->getPeer(pd0Name);
	if (pd0)
		pd0Controller = pd0->provides("pd")->getOperation("runController");
	
	pd1 = this->getPeer(pd1Name);
	if (pd1)
		pd1Controller = pd1->provides("pd")->getOperation("runController");
	
	pd2 = this->getPeer(pd2Name);
	if (pd2)
		pd2Controller = pd2->provides("pd")->getOperation("runController");
	
	pd3 = this->getPeer(pd3Name);
	if (pd3)
		pd3Controller = pd3->provides("pd")->getOperation("runController");
	
	pd4 = this->getPeer(pd4Name);
	if (pd4)
		pd4Controller = pd4->provides("pd")->getOperation("runController");
	
	pd5 = this->getPeer(pd5Name);
	if (pd5)
		pd5Controller = pd5->provides("pd")->getOperation("runController");
	
	P0 = pd0->properties()->getProperty("P");
	D0 = pd0->properties()->getProperty("D");
	P1 = pd1->properties()->getProperty("P");
	D1 = pd1->properties()->getProperty("D");
	P2 = pd2->properties()->getProperty("P");
	D2 = pd2->properties()->getProperty("D");
	P3 = pd3->properties()->getProperty("P");
	D3 = pd3->properties()->getProperty("D");
	P4 = pd4->properties()->getProperty("P");
	D4 = pd4->properties()->getProperty("D");
	P5 = pd5->properties()->getProperty("P");
	D5 = pd5->properties()->getProperty("D");
	
	log(Info) << "[ATCFLS] configured!" << endlog();
	return true;
}

bool ATCFastLegSwing::startHook() {
	log(Info) << "[ATCFLS] started!" << endlog();
	return true;
}

void ATCFastLegSwing::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCFastLegSwing::stopHook() {
	log(Info) << "[ATCFLS] stopped!" << endlog();
}

void ATCFastLegSwing::cleanupHook() {
	log(Info) << "[ATCFLS] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCFastLegSwing)

}
}
