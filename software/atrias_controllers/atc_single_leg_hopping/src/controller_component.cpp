/*! \file controller_component.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_single_leg_hopping controller.
 */

#include <atc_single_leg_hopping/controller_component.h>

// Initialize variables (IS THIS THE BEST PLACE FOR THIS??)
double timeInStance = 0.0;
bool isStance = false;

namespace atrias {
namespace controller {

ATCSingleLegHopping::ATCSingleLegHopping(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCSingleLegHopping::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add subcontrollers properties.
    this->addProperty("pd0Name", pd0Name)
        .doc("Leg PD subcontroller.");
    this->addProperty("pd1Name", pd1Name)
        .doc("Hip PD subcontroller.");

    // For the GUI
    addEventPort(guiDataIn);
    addPort(guiDataOut);
    pubTimer = new GuiPublishTimer(20);

    // Logging
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






    // begin control code //

    // Check if we are in flight or stance phase
    if (0.0 < 0.0){
        isStance = true; 
    } else {
        isStance = false;
    }

// DEBUG STUFF
    if (guiIn.debug1){
        isStance = true; 
    } else {
        isStance = false;
    }
// END DEBUG STUFF

    // Stance phase control
    if (isStance){

        // Counter
        timeInStance = timeInStance + 0.001;
        
        // Set gains
        P0.set(guiIn.stance_leg_P_gain);
        D0.set(guiIn.stance_leg_D_gain);
        P1.set(guiIn.stance_hip_P_gain);
        D1.set(guiIn.stance_hip_D_gain);

        // Define leg link lengths (thigh and shin).
        double l1 = 0.5;
        double l2 = 0.5;

        // Define spring constant.
        double k = 2450.0;

        // Define desired force at end-effector.
        double Fx = guiIn.slip_height;
        double Fy = guiIn.slip_mass;

        // Jacobian matrix with global joint angles.
        double J11 = l1*sin(rs.lLeg.halfA.motorAngle) + l2*sin(rs.lLeg.halfB.motorAngle);
        double J12 = -l2*sin(rs.lLeg.halfB.motorAngle);
        double J21 = l1*cos(rs.lLeg.halfA.motorAngle) + l2*cos(rs.lLeg.halfB.motorAngle);
        double J22 = l2*cos(rs.lLeg.halfB.motorAngle);

        // Compute required spring torque from desired end-effector force.
        double tauA = (Fx*J11 + Fy*J21);
        double tauB = (Fx*J12 + Fy*J22);

        // Compute required spring deflection.
        double desDeflA = tauA/k;
        double desDeflB = tauB/k;

        // Compute current spring deflection.
        double deflA = rs.lLeg.halfA.motorAngle - rs.lLeg.halfA.legAngle;
        double deflB = rs.lLeg.halfB.motorAngle - rs.lLeg.halfB.legAngle;

        // Compute required motor position.
        leftMotorAngle.A = rs.lLeg.halfA.legAngle + desDeflA;
        leftMotorAngle.B = rs.lLeg.halfB.legAngle + desDeflB;

        // Compute leg length from motor positions.
        double legAng = (rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle)/2.0;
        double legLen = cos(rs.lLeg.halfA.legAngle - legAng);

        // Make right leg mirror the left leg but with 85% of the length.
        rightMotorAngle = legToMotorPos(legAng, legLen*0.85);



    // Flight phase control
    } else {

        // Reset counter
        timeInStance = 0.0;

        // Set gains
        P0.set(guiIn.flight_leg_P_gain);
        D0.set(guiIn.flight_leg_D_gain);
        P1.set(guiIn.flight_hip_P_gain);
        D1.set(guiIn.flight_hip_D_gain);

        // Leg control
        leftMotorAngle = legToMotorPos(M_PI/2.0, 0.85);
        rightMotorAngle = legToMotorPos(M_PI/2.0, 0.85*0.85);

        // Hip control

    }

    // Set motor currents
    co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB = pd0Controller(leftMotorAngle.B, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
    co.lLeg.motorCurrentHip = pd1Controller(0.0, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentA = pd0Controller(rightMotorAngle.A, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB = pd0Controller(rightMotorAngle.B, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);      
    co.rLeg.motorCurrentHip = pd1Controller(0.0, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

    // end control code //





    // Command a run state
    co.command = medulla_state_run;

    // Let the GUI know the controller run state
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);

    // Send data to the GUI
    if (pubTimer->readyToSend())
        guiDataOut.write(guiOut);

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCSingleLegHopping::configureHook() {

    // Connect to the subcontrollers
    pd0 = this->getPeer(pd0Name);
    if (pd0)
        pd0Controller = pd0->provides("pd")->getOperation("runController");

    pd1 = this->getPeer(pd1Name);
    if (pd1)
        pd1Controller = pd1->provides("pd")->getOperation("runController");

    legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

    // Get references to the attributes
    P0 = pd0->properties()->getProperty("P");
    D0 = pd0->properties()->getProperty("D");
    P1 = pd1->properties()->getProperty("P");
    D1 = pd1->properties()->getProperty("D");

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCSingleLegHopping::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCSingleLegHopping::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCSingleLegHopping::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCSingleLegHopping::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCSingleLegHopping)

}
}
