/*! \file controller_component.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_single_leg_hopping controller.
 */

#include <atc_single_leg_hopping/controller_component.h>

// Initialize variables
bool isStance = false;

namespace atrias {
namespace controller {

// ATCSingleLegHopping ============================================================================
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
    this->addProperty("pd0Name", pd0Name).doc("Leg PD subcontroller.");
    this->addProperty("pd1Name", pd1Name).doc("Hip PD subcontroller.");

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


// ATCSingleLegHopping::runController =============================================================
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

    // BEGIN CONTROL CODE -------------------------------------------------------------------------

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


    // Stance phase control -----------------------------------------------------------------------
    if (isStance) {

        // Define constant force at end-effector.
        if (guiIn.constant_force) {
            Fx = guiIn.fx;
            Fz = guiIn.fz;

        // Define SLIP force trajectory at end-effector.
        } else if (guiIn.slip_force) {
            // Time step
            delta = 0.001;

            // SLIP model 4th orer Runge-Kutta numerical solution
            rNew = r + delta*(dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/3.0 + delta*dr/6.0 + delta*(dr + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass)/2.0)/3.0 + delta*(dr + delta*(-g*sin(-q - delta*(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)/2.0) + (r + delta*(dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0)*pow(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)*(2.0*dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr), 2) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*(dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0)/guiIn.slip_mass))/6.0;

            drNew = dr + delta*(g*sin(q + delta*(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)*(2.0*dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr))) + (r + delta*(dr + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass)/2.0))*pow(dq - delta*(g*cos(-q - delta*(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)/2.0) + (2.0*dr + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass))*(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)*(2.0*dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr)))/(r + delta*(dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0), 2) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*(dr + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass)/2.0))/guiIn.slip_mass)/6.0 + delta*(-g*sin(-q - delta*(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)/2.0) + (r + delta*(dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0)*pow(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)*(2.0*dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr), 2) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*(dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0)/guiIn.slip_mass)/3.0 + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/6.0 + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass)/3.0;

            qNew = q + delta*dq/6.0 + delta*(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)/3.0 + delta*(dq - delta*(g*cos(-q - delta*(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)/2.0) + (2.0*dr + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass))*(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)*(2.0*dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr)))/(r + delta*(dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0))/6.0 + delta*(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*(2.0*dq*dr + g*cos(q))/r/2.0)*(2.0*dr + delta*(r*dq*dq + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr))/3.0;

            dqNew = dq - delta*(g*cos(-q - delta*(dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0)/2.0) + ((2.0*dr) + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass))*(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0)*((2.0*dr) + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr)))/(3.0*r + 3.0/2.0*delta*(dr + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)) - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0)*((2.0*dr) + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(3.0*r + 3.0/2.0*delta*dr) - delta*(g*cos(q + delta*(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0)*((2.0*dr) + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr))) + ((2.0*dr) + 2.0*delta*(-g*sin(-q - delta*(dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0)/2.0) + (r + delta*(dr + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0)*pow(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0)*((2.0*dr) + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr), 2) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*(dr + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0)/guiIn.slip_mass))*(dq - delta*(g*cos(-q - delta*(dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0)/2.0) + ((2.0*dr) + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass))*(dq - delta*(g*cos(q + delta*dq/2.0) + (dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0)*((2.0*dr) + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)))/(2.0*r + delta*dr)))/(r + delta*(dr + delta*(r*(dq*dq) + g*sin(q) + guiIn.slip_spring*(guiIn.slip_leg - r)/guiIn.slip_mass)/2.0)/2.0)))/(6.0*r + 6.0*delta*(dr + delta*(g*sin(q + delta*dq/2.0) + pow(dq - delta*((2.0*dq*dr) + g*cos(q))/r/2.0, 2)*(r + delta*dr/2.0) - guiIn.slip_spring*(r - guiIn.slip_leg + delta*dr/2.0)/guiIn.slip_mass)/2.0)) - delta*((2.0*dq*dr) + g*cos(q))/r/6.0;

            // If virtual SLIP model is in flight.
            if (r > guiIn.slip_leg) {
                Fx = 0.0;
                Fz = 0.0;

            // If virtual SLIP model is in NOT in flight (is in stance).
            } else {
                // Replace last time step with next time step.
                r = rNew;
                dr = drNew;
                q = qNew;
                dq = dqNew;

                // Calculate force components
                Fx = guiIn.slip_spring*(guiIn.slip_leg - r)*sin(q);
                Fz = guiIn.slip_spring*(guiIn.slip_leg - r)*cos(q);
            }

        } else {
            Fx = 0.0;
            Fz = 0.0;

        }

        // DEBUG STATEMENTS .......................................................................
        if (guiIn.debug2) {
            printf("Fx: %f\n", Fx);
            printf("Fz: %f\n", Fz);
        }

        // Gains
        Ks = 1.6e4;
        Kg = 50.0;
        Kp = 97.3;
        Ki = 10.0;
        Kd = 3.3;

        // Compute required spring torque from desired end-effector force.
        tauA = Fz*l2*cos(rs.rLeg.halfA.legAngle + rs.position.bodyPitch - M_PI) + Fx*l2*sin(rs.rLeg.halfA.legAngle + rs.position.bodyPitch - M_PI);
        tauB = Fz*l1*cos(rs.rLeg.halfB.legAngle + rs.position.bodyPitch - M_PI) + Fx*l1*sin(rs.rLeg.halfB.legAngle + rs.position.bodyPitch - M_PI);

        // Compute required change in spring torque.
        dFx = 0.0;
        dFz = 0.0;

        dtauA = Fz*l1*sin(rs.rLeg.halfA.legAngle + rs.position.bodyPitch - M_PI)*(rs.rLeg.halfA.legVelocity + rs.position.bodyPitchVelocity) - dFx*l1*sin(rs.rLeg.halfA.legAngle + rs.position.bodyPitch - M_PI) - dFz*l1*cos(rs.rLeg.halfA.legAngle + rs.position.bodyPitch - M_PI) - Fx*l1*cos(rs.rLeg.halfA.legAngle + rs.position.bodyPitch - M_PI)*(rs.rLeg.halfA.legVelocity + rs.position.bodyPitchVelocity);

        dtauB = Fz*l2*sin(rs.rLeg.halfB.legAngle + rs.position.bodyPitch - M_PI)*(rs.rLeg.halfB.legVelocity + rs.position.bodyPitchVelocity) - dFx*l2*sin(rs.rLeg.halfB.legAngle + rs.position.bodyPitch - M_PI) - dFz*l2*cos(rs.rLeg.halfB.legAngle + rs.position.bodyPitch - M_PI) - Fx*l2*cos(rs.rLeg.halfB.legAngle + rs.position.bodyPitch - M_PI)*(rs.rLeg.halfB.legVelocity + rs.position.bodyPitchVelocity);

        beta1 = (rs.rLeg.halfB.legAngle - rs.rLeg.halfA.legAngle)/2.0;
        L1 = 2.0*l1*cos(beta1);
        alpha1 = rs.rLeg.halfB.legAngle - beta1 + rs.position.bodyPitch - M_PI;
        alpha2d = -alpha1;
        L2d = L1 * 0.8;
        beta2d = acos(L2d/2.0/l1);
        q7d = alpha2d - rs.position.bodyPitch - M_PI/2.0 + beta2d;
        q8d = alpha2d - rs.position.bodyPitch - M_PI/2.0 - beta2d;

        dbeta1 = (rs.rLeg.halfB.legVelocity - rs.rLeg.halfA.legVelocity)/2.0;
        dL1 = 2.0*l1*cos(dbeta1);
        dalpha1 = rs.rLeg.halfB.legVelocity - dbeta1 + rs.position.bodyPitchVelocity;
        dalpha2d = -dalpha1;
        dL2d = dL1 * 0.8;
        dbeta2d = -dL2d/sqrt(1.0 - pow((L2d/2.0/l1), 2));
        dq7d = dalpha2d - rs.position.bodyPitchVelocity + dbeta2d;
        dq8d = dalpha2d - rs.position.bodyPitchVelocity - dbeta2d;

        // Set stance leg gains and currents
        co.rLeg.motorCurrentA = Ks*(rs.rLeg.halfA.motorAngle - rs.rLeg.halfA.legAngle)/Kg + Kp*(tauA/Ks - (rs.rLeg.halfA.motorAngle - rs.rLeg.halfA.legAngle)) + Kd*(dtauA/Ks - (rs.rLeg.halfA.motorVelocity - rs.rLeg.halfA.legVelocity));
        co.rLeg.motorCurrentB = Ks*(rs.rLeg.halfB.motorAngle - rs.rLeg.halfB.legAngle)/Kg + Kp*(tauB/Ks - (rs.rLeg.halfB.motorAngle - rs.rLeg.halfB.legAngle)) + Kd*(dtauB/Ks - (rs.rLeg.halfB.motorVelocity - rs.rLeg.halfB.legVelocity));

        // Set flight leg gains and currents
        co.lLeg.motorCurrentA = Kp * (q8d - rs.lLeg.halfA.motorAngle + M_PI/2.0) + Kd * (dq8d - rs.lLeg.halfA.motorVelocity);
        co.lLeg.motorCurrentB = Kp * (q7d - rs.lLeg.halfB.motorAngle + M_PI/2.0) + Kd * (dq7d - rs.lLeg.halfB.motorVelocity);

        // DEBUG STATEMENTS .......................................................................
        if (guiIn.debug3) {
            printf("tauA: %f\n", tauA);
            printf("tauB: %f\n", tauB);
            printf("dtauA: %f\n", dtauA);
            printf("dtauB: %f\n", dtauB);
        }

    // Flight phase control -----------------------------------------------------------------------
    } else {
        
        // Reset initial conditions ...............................................................
        r = guiIn.slip_leg;
        dr = -sqrt(2.0*9.81*guiIn.slip_height);
        q = M_PI/2.0;
        dq = 0.0;

        // DEBUG STATEMENTS .......................................................................
        if (guiIn.debug4) {
            printf("r: %f\n", r);
            printf("dr: %f\n", dr);
            printf("q: %f\n", q);
            printf("dq: %f\n", dq);
        }

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

    // Hip Control --------------------------------------------------------------------------------
    if (guiIn.constant_hip) {
        // Constant angle
        leftHipAngle = guiIn.hip_angle;
        rightHipAngle = guiIn.hip_angle;

        // Set motor currents
        co.lLeg.motorCurrentHip = pd1Controller(leftHipAngle, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);     
        co.rLeg.motorCurrentHip = pd1Controller(rightHipAngle, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

    } else if (guiIn.advanced_hip){
        // Constant y end-effector position
        leftHipAngle = guiIn.left_hip_target;
        rightHipAngle = guiIn.right_hip_target;

        // Set motor currents
        co.lLeg.motorCurrentHip = pd1Controller(leftHipAngle, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);     
        co.rLeg.motorCurrentHip = pd1Controller(rightHipAngle, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

    } else {
        // Do nothing
    }

    // END CONTROL CODE ---------------------------------------------------------------------------

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
// configureHook ==================================================================================
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

// startHook ======================================================================================
bool ATCSingleLegHopping::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

// updateHook =====================================================================================
void ATCSingleLegHopping::updateHook() {
    guiDataIn.read(guiIn);
}

// stopHook =======================================================================================
void ATCSingleLegHopping::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

// cleanupHook ====================================================================================
void ATCSingleLegHopping::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCSingleLegHopping)

}
}
