/*! \file controller_component.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_single_leg_hopping controller.
 */

// Initialize ==================================================================
#include <atc_single_leg_hopping/controller_component.h>
bool isStance = false;

namespace atrias {
namespace controller {

// ATCSingleLegHopping =========================================================
ATCSingleLegHopping::ATCSingleLegHopping(std::string name):
    RTT::TaskContext(name), 
	logPort(name + "_log"), 
	guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in")
{

    this->provides("atc")->addOperation("runController", &ATCSingleLegHopping::runController, this, ClientThread).doc("Get robot_state from RTOps and return controller output.");

    // Add subcontrollers properties.
    this->addProperty("pd0Name", pd0Name).doc("Leg PD subcontroller.");
    this->addProperty("pd1Name", pd1Name).doc("Hip PD subcontroller.");

    // For the GUI
    addEventPort(guiDataIn);
    addPort(guiDataOut);
    pubTimer = new GuiPublishTimer(20);

    // Logging -----------------------------------------------------------------
    // Create a port
    addPort(logPort);
    // Buffer port so we capture all data.
    ConnPolicy policy = RTT::ConnPolicy::buffer(100000);
    // Transport type = ROS
    policy.transport = 3;
    // ROS topic name
    policy.name_id = "/" + name + "_log";
    // Construct the stream between the port and ROS topic
    logPort.createStream(policy);
    log(Info) << "[ATCMT] Constructed!" << endlog();
}


// ATCSingleLegHopping::runController ==========================================
atrias_msgs::controller_output ATCSingleLegHopping::runController(atrias_msgs::robot_state rs) {

    // Do nothing unless told otherwise
    co.lLeg.motorCurrentA   = 0.0;
    co.lLeg.motorCurrentB   = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA   = 0.0;
    co.rLeg.motorCurrentB   = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // Only run the controller when we're enabled
    if ((rtOps::RtOpsState)rs.rtOpsState != rtOps::RtOpsState::ENABLED)
        return co;


    // BEGIN CONTROL CODE ******************************************************

	// TODO - Temporary structures until I figure out how to pass the robot state to subcontrollers.
	rLeg.halfA.legAngle = rs.rLeg.halfA.legAngle;
	rLeg.halfB.legAngle = rs.rLeg.halfB.legAngle;
	rLeg.halfA.motorAngle = rs.rLeg.halfA.motorAngle;
	rLeg.halfB.motorAngle = rs.rLeg.halfB.motorAngle;
	rLeg.halfA.legVelocity = rs.rLeg.halfA.legVelocity;
	rLeg.halfB.legVelocity = rs.rLeg.halfB.legVelocity;
	rLeg.halfA.motorVelocity = rs.rLeg.halfA.motorVelocity;
	rLeg.halfB.motorVelocity = rs.rLeg.halfB.motorVelocity;
	lLeg.halfA.legAngle = rs.lLeg.halfA.legAngle;
	lLeg.halfB.legAngle = rs.lLeg.halfB.legAngle;
	position.bodyPitch = rs.position.bodyPitch;
	position.bodyPitchVelocity = rs.position.bodyPitchVelocity;
	position.boomAngle = rs.position.boomAngle;

    // Define leg link lengths (thigh and shin).
    l1 = 0.5;
    l2 = 0.5;

    // Gravity
    g = -9.81;

    // Check and set current state
    if (rs.position.zPosition < guiIn.slip_leg || guiIn.debug1) {
        isStance = true;
    } else {
        isStance = false;
    }


    // Stance phase control ----------------------------------------------------
    if (isStance) {

        // Define constant force at end-effector.
        if (guiIn.constant_force) {
            fx = guiIn.fx;
            fz = guiIn.fz;
        // Define SLIP force trajectory at end-effector.
        } else if (guiIn.slip_force) {            
			// SLIP FORCE STUFF
        } else {
            fx = 0.0;
            fz = 0.0;
        }

		// Force control leg
		legForce.fx = fx;
		legForce.fz = fz;
		legForce.dfx = 0.0;
		legForce.dfz = 0.0;
		motorCurrent = legForceToMotorCurrent(legForce, rLeg, position);
        co.rLeg.motorCurrentA = motorCurrent.A;
        co.rLeg.motorCurrentB = motorCurrent.B;

		// Swing leg control
        Kpidf1 = guiIn.flight_leg_P_gain;
        Kpidf3 = guiIn.flight_leg_D_gain;
        qs1 = rs.rLeg.halfB.legAngle - M_PI/2.0;
        qs2 = rs.rLeg.halfA.legAngle - M_PI/2.0;
        qs7 = rs.lLeg.halfB.motorAngle - M_PI/2.0;
        qs8 = rs.lLeg.halfA.motorAngle - M_PI/2.0;
        qs9 = rs.position.bodyPitch - 3.0*M_PI/2.0;
        qs9 = fmod(qs9 + 4.0*M_PI, 2.0*M_PI);
        qsdot7 = rs.lLeg.halfB.motorVelocity;
        qsdot8 = rs.lLeg.halfA.motorVelocity;
        qsdot9 = rs.position.bodyPitchVelocity;

        beta1 = (qs1 - qs2)/2.0;
        L1 = (l1 + l2)*cos(beta1);
        alpha1 = qs1 - beta1 + qs9;
        alpha2d = -alpha1;
        L2d = L1*0.8;
        beta2d = acos(L2d/2.0/l1);
        q7d = alpha2d - qs9 + beta2d;
        q8d = alpha2d - qs9 - beta2d;

        Dbeta1 = (qsdot1 - qsdot2)/2.0;
        DL1 = (l1 + l2)*cos(Dbeta1);
        Dalpha1 = qsdot1 - Dbeta1 + qsdot9;
        Dalpha2d = -Dalpha1;
        DL2d = DL1 * 0.8;
        Dbeta2d = -DL2d/sqrt(1.0 - pow((L2d/2.0/l1), 2));
        Dq7d = Dalpha2d - qsdot9 + Dbeta2d;
        Dq8d = Dalpha2d - qsdot9 - Dbeta2d;

        Tm3 = Kpidf1*(q7d - qs7) + Kpidf3*(Dq7d - qsdot7);
        Tm4 = Kpidf1*(q8d - qs8) + Kpidf3*(Dq8d - qsdot8);

        co.lLeg.motorCurrentB = Tm3*0.0;
        co.lLeg.motorCurrentA = Tm4*0.0;

    // Flight phase control ----------------------------------------------------
    } else {
        
        // Reset initial conditions ............................................
        r = guiIn.slip_leg;
        dr = -sqrt(2.0*fabs(g)*guiIn.slip_height);
        q = M_PI/2.0;
        dq = 0.0;

        // Leg control
        leftMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_leg*0.8);
        rightMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_leg);

        // Set hip gains
        P1.set(guiIn.flight_hip_P_gain);
        D1.set(guiIn.flight_hip_D_gain);

        // Set flight leg gains and currents
        P0.set(guiIn.flight_leg_P_gain);
        D0.set(guiIn.flight_leg_D_gain);
        co.rLeg.motorCurrentA = pd0Controller(rightMotorAngle.A, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
        co.rLeg.motorCurrentB = pd0Controller(rightMotorAngle.B, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity); 
        co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
        co.lLeg.motorCurrentB = pd0Controller(leftMotorAngle.B, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);

    }

    // Hip Control -------------------------------------------------------------
    if (guiIn.constant_hip) {
        // Constant angle
        hipAngle.left = guiIn.hip_angle;
        hipAngle.right = guiIn.hip_angle;

        // Set motor currents
        co.lLeg.motorCurrentHip = pd1Controller(hipAngle.left, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = pd1Controller(hipAngle.right, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

    } else if (guiIn.advanced_hip){
		// HipInverseKinematics subcontroller
		toePosition.left = guiIn.left_hip_target;
		toePosition.right = guiIn.right_hip_target;
		hipAngle = toePositionToHipAngle(toePosition, lLeg, rLeg, position);

        // Set motor currents
        co.lLeg.motorCurrentHip = pd1Controller(hipAngle.left, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);     
        co.rLeg.motorCurrentHip = pd1Controller(hipAngle.right, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

    } else {
        // Do nothing
    }


	// END CONTROL CODE ********************************************************

    // Command a run state
    co.command = medulla_state_run;

    // Let the GUI know the controller run state
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);

    // Send data to the GUI
    if (pubTimer->readyToSend()) guiDataOut.write(guiOut);

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;

} // ATCSingleLegHopping::runController

// DON'T PUT CONTROL CODE BELOW HERE! ******************************************

// configureHook ===============================================================
bool ATCSingleLegHopping::configureHook() {

    // ASCPD Service
    pd0 = this->getPeer(pd0Name);
    if (pd0) pd0Controller = pd0->provides("pd")->getOperation("runController");
    P0 = pd0->properties()->getProperty("P");
    D0 = pd0->properties()->getProperty("D");

	// ASCPD Service
    pd1 = this->getPeer(pd1Name);
    if (pd1) pd1Controller = pd1->provides("pd")->getOperation("runController");
    P1 = pd1->properties()->getProperty("P");
    D1 = pd1->properties()->getProperty("D");

	// ASCLegToMotorTransforms Service
    legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

	// ASCLegForce Service
	legForceToMotorCurrent = this->provides("ascLegForce")->getOperation("legForceToMotorCurrent");

	// ASCHipInverseKinematics Service
	toePositionToHipAngle = this->provides("ascHipInverseKinematics")->getOperation("toePositionToHipAngle");

    log(Info) << "[ATCMT] configured!" << endlog();

    return true;
} // configureHook

// startHook ===================================================================
bool ATCSingleLegHopping::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
} // startHook

// updateHook ==================================================================
void ATCSingleLegHopping::updateHook() {
    guiDataIn.read(guiIn);
} // updateHook

// stopHook ====================================================================
void ATCSingleLegHopping::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
} // stopHook

// cleanupHook =================================================================
void ATCSingleLegHopping::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
} // cleanupHook

ORO_CREATE_COMPONENT(ATCSingleLegHopping)

} // namespace controller
} // namespace atrias
