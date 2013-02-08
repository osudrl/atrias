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
                Fx = -guiIn.slip_spring*(guiIn.slip_leg - r)*cos(q);
                Fz = -guiIn.slip_spring*(guiIn.slip_leg - r)*sin(q);
            }

        } else {
            Fx = 0.0;
            Fz = 0.0;

        }

        // Variable conversion for Benham coordinates
        Fx = Fx;
        Fy = -Fz;
        Fxdot = 0.0;
        Fydot = 0.0;
        Ks = guiIn.robot_spring;
        Kg = 50.0;        
        Kpid1 = guiIn.stance_leg_P_gain;
        Kpid3 = guiIn.stance_leg_D_gain;
        Kpidf1 = guiIn.flight_leg_P_gain;
        Kpidf3 = guiIn.flight_leg_D_gain;
        qs1 = rs.rLeg.halfB.legAngle - M_PI/2.0;
        qs2 = rs.rLeg.halfA.legAngle - M_PI/2.0;
        qs3 = rs.rLeg.halfB.motorAngle - M_PI/2.0;
        qs4 = rs.rLeg.halfA.motorAngle - M_PI/2.0;
        qs5 = rs.lLeg.halfB.legAngle - M_PI/2.0; 
        qs6 = rs.lLeg.halfA.legAngle - M_PI/2.0;
        qs7 = rs.lLeg.halfB.motorAngle - M_PI/2.0;
        qs8 = rs.lLeg.halfA.motorAngle - M_PI/2.0;
        qs9 = rs.position.bodyPitch - 3.0*M_PI/2.0;
        qsdot1 = rs.rLeg.halfB.legVelocity;
        qsdot2 = rs.rLeg.halfA.legVelocity;
        qsdot3 = rs.rLeg.halfB.motorVelocity;
        qsdot4 = rs.rLeg.halfA.motorVelocity;
        qsdot5 = rs.lLeg.halfB.legVelocity; 
        qsdot6 = rs.lLeg.halfA.legVelocity;
        qsdot7 = rs.lLeg.halfB.motorVelocity;
        qsdot8 = rs.lLeg.halfA.motorVelocity;
        qsdot9 = rs.position.bodyPitchVelocity;

        Ts_d1 = -Fx*l1*cos(qs1 + qs9) - Fy*l1*sin(qs1 + qs9);
        Ts_d2 = -Fx*l2*cos(qs2 + qs9) - Fy*l2*sin(qs2 + qs9);

        Tsdot_d1 = Fx*l1*sin(qs1 + qs9)*(qsdot1 + qsdot9) - Fydot*l1*sin(qs1 + qs9) - Fxdot*l1*cos(qs1 + qs9) - Fy*l1*cos(qs1 + qs9)*(qsdot1 + qsdot9);
        Tsdot_d2 = Fx*l2*sin(qs2 + qs9)*(qsdot2 + qsdot9) - Fydot*l2*sin(qs2 + qs9) - Fxdot*l2*cos(qs2 + qs9) - Fy*l2*cos(qs2 + qs9)*(qsdot2 + qsdot9);

        Tm1 = Ks*(qs3 - qs1)/Kg + Kpid1*(Ts_d1/Ks - (qs3 - qs1)) + Kpid3*(Tsdot_d1/Ks - (qsdot3 - qsdot1));
        Tm2 = Ks*(qs4 - qs2)/Kg + Kpid1*(Ts_d2/Ks - (qs4 - qs2)) + Kpid3*(Tsdot_d2/Ks - (qsdot4 - qsdot2));

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

        co.rLeg.motorCurrentB = Tm1;
        co.rLeg.motorCurrentA = Tm2;
        co.lLeg.motorCurrentB = Tm3;
        co.lLeg.motorCurrentA = Tm4;

        if (guiIn.debug2) {
            printf("qs1: %f\n", qs1);
            printf("qs2: %f\n", qs2);
            printf("qs3: %f\n", qs3);
            printf("qs4: %f\n", qs4);
            printf("qs5: %f\n", qs5);
            printf("qs6: %f\n", qs6);
            printf("qs7: %f\n", qs7);
            printf("qs8: %f\n", qs8);
            printf("qs9: %f\n", qs9);
            printf("qsdot1: %f\n", qsdot1);
            printf("qsdot2: %f\n", qsdot2);
            printf("qsdot3: %f\n", qsdot3);
            printf("qsdot4: %f\n", qsdot4);
            printf("qsdot5: %f\n", qsdot5);
            printf("qsdot6: %f\n", qsdot6);
            printf("qsdot7: %f\n", qsdot7);
            printf("qsdot8: %f\n", qsdot8);
            printf("qsdot9: %f\n", qsdot9);
            printf("Ts_d1: %f\n", Ts_d1);
            printf("Ts_d2: %f\n", Ts_d2);
            printf("Tsdot_d1: %f\n", Tsdot_d1);
            printf("Tsdot_d2: %f\n", Tsdot_d2);
            printf("Tm1: %f\n", Tm1);
            printf("Tm2: %f\n", Tm2);
            printf("beta1: %f\n", beta1);
            printf("L1: %f\n", L1);
            printf("aplha1: %f\n", alpha1);
            printf("alpha2d: %f\n", alpha2d);
            printf("L2d: %f\n", L2d);
            printf("beta2d: %f\n", beta2d);
            printf("q7d: %f\n", q7d);
            printf("q8d: %f\n", q8d);
            printf("Dbeta1: %f\n", Dbeta1);
            printf("DL1: %f\n", DL1);
            printf("Daplha1: %f\n", Dalpha1);
            printf("Dalpha2d: %f\n", Dalpha2d);
            printf("DL2d: %f\n", DL2d);
            printf("Dbeta2d: %f\n", Dbeta2d);
            printf("Dq7d: %f\n", Dq7d);
            printf("Dq8d: %f\n", Dq8d);
            printf("Tm3: %f\n", Tm3);
            printf("Tm4: %f\n", Tm4);
        }


    // Flight phase control -----------------------------------------------------------------------
    } else {
        
        // Reset initial conditions ...............................................................
        r = guiIn.slip_leg;
        dr = -sqrt(2.0*9.81*guiIn.slip_height);
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

    // Hip Control --------------------------------------------------------------------------------
    if (guiIn.constant_hip) {
        // Constant angle
        leftHipAngle = guiIn.hip_angle;
        rightHipAngle = guiIn.hip_angle;

        // Set motor currents
        co.lLeg.motorCurrentHip = pd1Controller(leftHipAngle, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = pd1Controller(rightHipAngle, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

    } else if (guiIn.advanced_hip){
        // Define imaginary number i.
        i = complex<double>(0.0, 1.0);

        // Determine hip angles from desired toe positions.
        lBoom = 3.0;
        lBody = 0.5;
        lHip = 0.1;
        qBodyOffset = M_PI/2.0;
        qBoom = rs.position.boomAngle;
        lLeftLeg = (l1 + l2)*cos((rs.lLeg.halfB.legAngle - rs.lLeg.halfA.legAngle)/2.0);
        lRightLeg = (l1 + l2)*cos((rs.rLeg.halfB.legAngle - rs.rLeg.halfA.legAngle)/2.0);
        qLeftLeg = (rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle)/2.0;
        qRightLeg = (rs.rLeg.halfA.legAngle + rs.rLeg.halfB.legAngle)/2.0;
        leftToeRadius = guiIn.left_hip_target;
        rightToeRadius = guiIn.right_hip_target;

        leftHipAngleComplex = - qBoom - qBodyOffset - log((- sqrt(pow(lLeftLeg, 2) - 2.0*pow(lLeftLeg, 2)*exp(qLeftLeg*2.0*i) + pow(lLeftLeg, 2)*exp(qLeftLeg*4.0*i) + 4.0*pow(leftToeRadius, 2)*exp(qLeftLeg*2.0*i) - 4.0*pow(lHip, 2)*exp(qLeftLeg*2.0*i) + 4.0*pow(lBoom, 2)*exp(qLeftLeg*2.0*i)*pow(cos(qBoom), 2) - 4.0*pow(lLeftLeg, 2)*exp(qLeftLeg*2.0*i)*pow(cos(qLeftLeg), 2) + 4.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*pow(cos(qBoom), 2)*pow(cos(qBodyOffset), 2) + 4.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*pow(sin(qBoom), 2)*pow(sin(qBodyOffset), 2) + 8.0*lBoom*exp(qLeftLeg*2.0*i)*cos(qBoom)*sqrt(leftToeRadius + lLeftLeg*cos(qLeftLeg))*sqrt(leftToeRadius - lLeftLeg*cos(qLeftLeg)) + 8.0*lBoom*lBody*exp(qLeftLeg*2.0*i)*pow(cos(qBoom), 2)*cos(qBodyOffset) - 8.0*lBoom*lBody*exp(qLeftLeg*2.0*i)*cos(qBoom)*sin(qBoom)*sin(qBodyOffset) + 8.0*lBody*exp(qLeftLeg*2.0*i)*cos(qBoom)*cos(qBodyOffset)*sqrt(leftToeRadius + lLeftLeg*cos(qLeftLeg))*sqrt(leftToeRadius - lLeftLeg*cos(qLeftLeg)) - 8.0*lBody*exp(qLeftLeg*2.0*i)*sin(qBoom)*sin(qBodyOffset)*sqrt(leftToeRadius + lLeftLeg*cos(qLeftLeg))*sqrt(leftToeRadius - lLeftLeg*cos(qLeftLeg)) - 8.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*cos(qBoom)*cos(qBodyOffset)*sin(qBoom)*sin(qBodyOffset)) + 2.0*lBoom*cos(qBoom)*(cos(qLeftLeg) + sin(qLeftLeg)*i) + 2.0*exp(qLeftLeg*i)*sqrt(leftToeRadius + lLeftLeg*cos(qLeftLeg))*sqrt(leftToeRadius - lLeftLeg*cos(qLeftLeg)) + 2.0*lBody*cos(qBoom + qBodyOffset)*(cos(qLeftLeg) + sin(qLeftLeg)*i))/(lLeftLeg + 2.0*lHip*exp(qLeftLeg*i) - lLeftLeg*exp(qLeftLeg*2.0*i)))*i;

        rightHipAngleComplex = - qBoom - qBodyOffset - log(-(- sqrt(2.0*pow(lBody, 2)*exp(qRightLeg*2.0*i) - 4.0*pow(lHip, 2)*exp(qRightLeg*2.0*i) - 2.0*pow(lRightLeg, 2)*exp(qRightLeg*2.0*i) + pow(lRightLeg, 2)*exp(qRightLeg*4.0*i) + 4.0*pow(rightToeRadius, 2)*exp(qRightLeg*2.0*i) + pow(lRightLeg, 2) + 4.0*pow(lBoom, 2)*exp(qRightLeg*2.0*i)*pow(cos(qBoom), 2) - 4.0*pow(lRightLeg, 2)*exp(qRightLeg*2.0*i)*pow(cos(qRightLeg), 2) + 2.0*pow(lBody, 2)*cos(2.0*qBoom + 2.0*qBodyOffset)*exp(qRightLeg*2.0*i) + 4.0*lBoom*lBody*exp(qRightLeg*2.0*i)*cos(qBodyOffset) + 4.0*lBoom*lBody*cos(2.0*qBoom + qBodyOffset)*exp(qRightLeg*2.0*i) + 8.0*lBody*exp(qRightLeg*2.0*i)*cos(qBoom + qBodyOffset)*sqrt(rightToeRadius + lRightLeg*cos(qRightLeg))*sqrt(rightToeRadius - lRightLeg*cos(qRightLeg)) + 8.0*lBoom*exp(qRightLeg*2.0*i)*cos(qBoom)*sqrt(rightToeRadius + lRightLeg*cos(qRightLeg))*sqrt(rightToeRadius - lRightLeg*cos(qRightLeg))) + 2.0*exp(qRightLeg*i)*sqrt(rightToeRadius + lRightLeg*cos(qRightLeg))*sqrt(rightToeRadius - lRightLeg*cos(qRightLeg)) + 2.0*lBody*exp(qRightLeg*i)*cos(qBoom + qBodyOffset) + 2.0*lBoom*exp(qRightLeg*i)*cos(qBoom))/(- lRightLeg + 2.0*lHip*exp(qRightLeg*i) + lRightLeg*exp(qRightLeg*2.0*i)))*i;

        // Only care about real part, imaginary part should be zero anyways but just in case.
        leftHipAngle = -real(leftHipAngleComplex) + M_PI/2.0;
        rightHipAngle = -real(rightHipAngleComplex) + M_PI/2.0;
        leftHipAngle = fmod(leftHipAngle, 2.0*M_PI);
        rightHipAngle = fmod(rightHipAngle, 2.0*M_PI);

        // DEBUG STATEMENTS .......................................................................
        if (guiIn.debug4) {
            printf("leftHipAngle: %f\n", leftHipAngle);
            printf("rightHipAngle: %f\n", rightHipAngle);
        }

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
