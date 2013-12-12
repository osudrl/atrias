/**
  * @file ATCEqPoint.cpp
  * @author Daniel Renjewski and Andrew Peekema
  * @brief Controller code for the atc_eq_point controller.
  */

#include "atc_eq_point/ATCEqPoint.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCEqPoint::ATCEqPoint(string name) :
    ATC(name),
    commonToolkit(this, "commonToolkit"),
    hipKine(this, "hipKine"),
    pd0Controller(this, "pd0Controller"),
    pd1Controller(this, "pd1Controller"),
    pd2Controller(this, "pd2Controller"),
    pd3Controller(this, "pd3Controller"),
    pd4Controller(this, "pd4Controller"),
    pd5Controller(this, "pd5Controller"),
    pdTorsoController(this, "pdTorsoController")
{
    // Initial PD controller gains
    legP = 600;
    legD = 15;
    // TODO: Revert for physical robot testing
    //hipP = 150;
    //hipD = 10;
    hipP = 0;
    hipD = 0;
    pd0Controller.P = legP;
    pd1Controller.P = legP;
    pd2Controller.P = hipP;
    pd3Controller.P = legP;
    pd4Controller.P = legP;
    pd5Controller.P = hipP;
    pd0Controller.D = legD;
    pd1Controller.D = legD;
    pd2Controller.D = hipD;
    pd3Controller.D = legD;
    pd4Controller.D = legD;
    pd5Controller.D = hipD;
    pdTorsoController.P = legP;
    pdTorsoController.D = legD;

    // Initialize state
    state = 0;

    // Slowly increase the current limit to maximum
    setStartupEnabled(true);
}

void ATCEqPoint::controller() {
    MotorState desiredLAState,
             desiredLBState,
             desiredRAState,
             desiredRBState;

    // Handle idle mode
    if (!isEnabled()) {
        // We're disabled... leave idle mode
        idle_mode = false;
    }
    if (guiIn.control == 0) {
        // GUI is commanding idle mode
        idle_mode = true;
    }

    // Run the idle mode controller
    if (idle_mode) {
        // Set the hip controller P gains to 0 for a gentle relaxation.
        pd2Controller.P = (0.0);
        pd5Controller.P = (0.0);
        co.lLeg.motorCurrentHip = pd2Controller(0.0, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = pd5Controller(0.0, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

        return;
    } else {
        // Revert the hip gains
        pd2Controller.P = hipP;
        pd5Controller.P = hipP;
    }

    // Only run the controller when we're enabled
    if (!isEnabled()) {
        return;
    }


    ///// begin control code /////

    // map GUI buttons for GC (Simulation)
    rGC = guiIn.gc_r;
    lGC = guiIn.gc_l;

    // read toeswitch from robot
    //rGC=rs.rLeg.toeSwitch;
    //lGC=rs.lLeg.toeSwitch;


    // initial state
    // Calculate motor angles
    MotorAngle leftMotorAngle;
    std::tie(leftMotorAngle.A,  leftMotorAngle.B)  = commonToolkit.legPos2MotorPos(guiIn.pea, guiIn.lst);
    MotorAngle rightMotorAngle;
    std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(guiIn.aea, guiIn.lst);

    // Hip control
    LeftRight hipTgts;
    LeftRight toePosition;
    toePosition.left  = guiIn.lhip_pos;
    toePosition.right = guiIn.rhip_pos;;
    std::tie(hipTgts.left, hipTgts.right) = hipKine.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
    co.lLeg.motorCurrentHip = pd2Controller(hipTgts.left,  rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pd5Controller(hipTgts.right, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

    // Smoothly initialize and set state to 2
    if (state == 0) {
        // Desired leg motor positions
        desiredLAState.ang = leftMotorAngle.A;
        desiredLAState.vel = 0.0;
        desiredLBState.ang = leftMotorAngle.B;
        desiredLBState.vel = 0.0;
        desiredRAState.ang = rightMotorAngle.A;
        desiredRAState.vel = 0.0;
        desiredRBState.ang = rightMotorAngle.B;
        desiredRBState.vel = 0.0;

        // Run the PD controllers
        co.lLeg.motorCurrentA = pd0Controller(desiredLAState.ang, rs.lLeg.halfA.motorAngle, desiredLAState.vel, rs.lLeg.halfA.motorVelocity);
        co.lLeg.motorCurrentB = pd1Controller(desiredLBState.ang, rs.lLeg.halfB.motorAngle, desiredLBState.vel, rs.lLeg.halfB.motorVelocity);
        co.rLeg.motorCurrentA = pd3Controller(desiredRAState.ang, rs.rLeg.halfA.motorAngle, desiredRAState.vel, rs.rLeg.halfA.motorVelocity);
        co.rLeg.motorCurrentB = pd4Controller(desiredRBState.ang, rs.rLeg.halfB.motorAngle, desiredRBState.vel, rs.rLeg.halfB.motorVelocity);

        // Change state if startup is no longer occurring
        if (!isStarting()) {
            //printf("Startup is finished\n");
            state = 2;
            sw_flight = true;
            sw_stance = true;
            t=1;
        }
        return;
    }

    // calculate leg angle and length
    amp=guiIn.lst-guiIn.lfl;
    phi_rLeg = (rs.rLeg.halfA.motorAngle+rs.rLeg.halfB.motorAngle)/2;                //right leg angle
    phi_lLeg = (rs.lLeg.halfA.motorAngle+rs.lLeg.halfB.motorAngle)/2;                //left leg angle


    // calculate circle correction for step length
    gamma = guiIn.pea - guiIn.aea;
    oPEA = guiIn.pea;
    iPEA = guiIn.pea - guiIn.rco * 0.09 * gamma;
    oAEA = guiIn.aea;
    iAEA = guiIn.aea + guiIn.rco * 0.09 * gamma;
    phiBs_des_i = iPEA + acos (guiIn.lst); //desired motor position for stance MOTOR B
    phiBs_des_o = oPEA + acos (guiIn.lst); //desired motor position for stance MOTOR B


    // Manual or automatic switching?
    switch (guiIn.control)
    {
        case 1: // Manual switch left / right stepping
            if (state == 2 && t>0.98 && rGC) 
            {
                state = 1;
                sw_stance = false;
                sw_flight = false;
                t=0;
            }
            if (state == 1 && t>0.98 && lGC)
            {
                state = 2;
                sw_stance = false;
                sw_flight = false;
                t=0;
            }
            break;
        case 2: // Automatic switch left / right stepping
            //if (state == 2 && phi_rLeg<M_PI/2 && rGC)
            if ((state == 2) && (t>0.98))
            {
                state = 1;
                sw_stance = false;
                sw_flight = false;
                t=0;
            }
            //if (state == 1 && phi_lLeg < M_PI/2 && lGC)
            if ((state == 1) && (t>0.98))
            {
                state = 2;
                sw_stance = false;
                sw_flight = false;
                t=0;
            }
            break;
        default:
            break;
    }

    //printf("t: %f \n",t);
    // command legs ------------------------------------------------
    //*stance control right leg*
    switch (state)  //stance leg right, flight leg left
    {
        case 1:
            s = 1 - (oPEA - phi_rLeg) / (oPEA - oAEA);
            if (s > t)
            {
                t=s;
            }
            pd3Controller.D = guiIn.dst;
            pd3Controller.P = guiIn.pst;
            pd4Controller.D = guiIn.dst;
            pd4Controller.P = guiIn.pst;
            pdTorsoController.P = guiIn.pst;
            pdTorsoController.D = guiIn.dst;
            // Stance leg control
            if ((rs.rLeg.halfB.motorAngle < phiBs_des_o) && !sw_stance)
            {   // Rotate to pea
                // asymmetry - extend right leg
                std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(phi_rLeg,guiIn.lst);
                co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0.5,rs.rLeg.halfB.motorVelocity) + (1 - guiIn.tab) * guiIn.thip;
                co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0.5,rs.rLeg.halfA.motorVelocity) - guiIn.tab * guiIn.thip;
            } else
            {   // If pea was reached once
                sw_stance=true;
                std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(guiIn.pea, guiIn.lst);
                co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity);
                co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
            }

            // During stance, move the the torso to a desired angle (3*pi/2 == vertical)
            torsoTorque = pdTorsoController(guiIn.desiredTorsoAngle,rs.position.bodyPitch,0,rs.position.bodyPitchVelocity);
            // Distribute torque between the two motors
            co.rLeg.motorCurrentA += guiIn.tab*torsoTorque;
            co.rLeg.motorCurrentB += (1 - guiIn.tab)*torsoTorque;
            // TODO: Add desired torso angle to gui


            //*flight control left leg*
            if ((t < 1) && (!sw_flight))
            {
                if (t < guiIn.lot){  // Liftoff time (Takeoff)
                    pd0Controller.P = guiIn.pfl1;
                    pd0Controller.D = guiIn.dfl1;
                    pd1Controller.P = guiIn.pfl1;
                    pd1Controller.D = guiIn.dfl1;
                    l_swing = guiIn.lst - amp *  ((-1) * pow(2.0 * t/guiIn.tsw - 1.0, 2.0) + 1.0);
                    phi_lLeg = iPEA;
                    logOut.state=11;
                } else if (t<guiIn.tsw){  // Swing phase
                    pd0Controller.P = guiIn.pfl1;
                    pd0Controller.D = guiIn.dfl1;
                    pd1Controller.P = guiIn.pfl1;
                    pd1Controller.D = guiIn.dfl1;
                    phi_lLeg=iPEA - (t - guiIn.lot) / (guiIn.tsw - guiIn.lot) * (iPEA - iAEA) * (1 + guiIn.aover);
                    l_swing = guiIn.lst -  amp *  ((-1) * pow(2.0 * t/guiIn.tsw - 1.0, 2.0) + 1.0);
                    logOut.state=12;
                } else {  // Retraction
                    pd0Controller.P = guiIn.pfl2;
                    pd0Controller.D = guiIn.dfl2;
                    pd1Controller.P = guiIn.pfl2;
                    pd1Controller.D = guiIn.dfl2;
                    phi_lLeg=guiIn.aea - (1 - t) / (1 - guiIn.tsw) * (iPEA-iAEA) * guiIn.aover;
                    l_swing = guiIn.lst;
                    logOut.state=13;
                }

                //map leg angle sweep of flight leg to 0->1
                std::tie(leftMotorAngle.A, leftMotorAngle.B) = commonToolkit.legPos2MotorPos(phi_lLeg,l_swing);
                co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
                co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity);
                if ((t < guiIn.lot) & (leftMotorAngle.B < (rs.lLeg.halfB.legAngle - (rs.lLeg.halfA.motorAngle - rs.lLeg.halfB.legAngle))))
                {
                    co.lLeg.motorCurrentB = guiIn.loc;
                    logOut.state=11.1;
                }

            } else
            {   // aea is reached once
                sw_flight=true;
                std::tie(leftMotorAngle.A, leftMotorAngle.B) = commonToolkit.legPos2MotorPos(iAEA,guiIn.lst);
                pd0Controller.D = guiIn.dst;
                pd0Controller.P = guiIn.pst;
                co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
                pd1Controller.D = guiIn.dst;
                pd1Controller.P = guiIn.pst;
                co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity);
                logOut.state=14;
            }
            break;


        //**************************************************************
        case 2:  // stance leg left, swing leg right
            s = 1 - (iPEA - phi_lLeg) / (iPEA - iAEA);
            if (s > t)
            {
                t=s;
            }
            pd0Controller.D = guiIn.dst;
            pd0Controller.P = guiIn.pst;
            pd1Controller.D = guiIn.dst;
            pd1Controller.P = guiIn.pst;
            pdTorsoController.P = guiIn.pst;
            pdTorsoController.D = guiIn.dst;
            if ((rs.lLeg.halfB.motorAngle < phiBs_des_i) && !sw_stance)
            { // stance leg rotate to pea
                std::tie(leftMotorAngle.A, leftMotorAngle.B) = commonToolkit.legPos2MotorPos(phi_lLeg,guiIn.lst);
                co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0.5,rs.lLeg.halfB.motorVelocity) + (1 - guiIn.tab) * guiIn.thip;
                co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0.5,rs.lLeg.halfA.motorVelocity) - guiIn.tab * guiIn.thip;
            } else
            { // if aea was reached once
                sw_stance=true;
                std::tie(leftMotorAngle.A, leftMotorAngle.B) = commonToolkit.legPos2MotorPos(guiIn.pea,guiIn.lst);
                co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity);
                co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
                //logOut.currState = 4;
            }

            // During stance, move the the torso to a desired angle (3*pi/2 == vertical)
            torsoTorque = pdTorsoController(guiIn.desiredTorsoAngle,rs.position.bodyPitch,0,rs.position.bodyPitchVelocity);
            // Distribute torque between the two motors
            co.lLeg.motorCurrentA += guiIn.tab*torsoTorque;
            co.lLeg.motorCurrentB += (1 - guiIn.tab)*torsoTorque;

            //******************************************************
            if ((t < 1) && (!sw_flight))
            {
                if (t < guiIn.lot){
                    pd3Controller.P = guiIn.pfl1;
                    pd3Controller.D = guiIn.dfl1;
                    pd4Controller.P = guiIn.pfl1;
                    pd4Controller.D = guiIn.dfl1;
                    l_swing = guiIn.lst - amp *  ((-1) * pow(2.0 * t/guiIn.tsw - 1.0, 2.0) + 1.0);
                    phi_rLeg = oPEA;
                    logOut.state=21;
                } else if (t<guiIn.tsw){
                    pd3Controller.P = guiIn.pfl1;
                    pd3Controller.D = guiIn.dfl1;
                    pd4Controller.P = guiIn.pfl1;
                    pd4Controller.D = guiIn.dfl1;
                    phi_rLeg=oPEA - (t - guiIn.lot) / (guiIn.tsw - guiIn.lot) * (oPEA - oAEA) * (1 + guiIn.aover);
                    l_swing = guiIn.lst - amp *  ((-1) * pow(2.0 * t/guiIn.tsw - 1.0, 2.0) + 1.0);
                    logOut.state=22;
                } else {
                    pd3Controller.P = guiIn.pfl2;
                    pd3Controller.D = guiIn.dfl2;
                    pd4Controller.P = guiIn.pfl2;
                    pd4Controller.D = guiIn.dfl2;
                    phi_rLeg=oAEA - (1 - t) / (1 - guiIn.tsw) * (oPEA-oAEA) * guiIn.aover;
                    l_swing = guiIn.lst;
                    logOut.state=23;
                }

                //map leg angle sweep of flight leg to 0->1
                std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(phi_rLeg,l_swing);
                co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
                co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity);
                if ((t < guiIn.lot) & (rightMotorAngle.B < (rs.rLeg.halfB.legAngle - (rs.rLeg.halfA.motorAngle - rs.rLeg.halfB.legAngle))))
                {
                    co.rLeg.motorCurrentB=guiIn.loc;
                    logOut.state=21.1;
                }

            } else {   // aea is reached once
                sw_flight=true;
                std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(oAEA,guiIn.lst);
                pd3Controller.D = guiIn.dst;
                pd3Controller.P = guiIn.pst;
                co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
                pd4Controller.D = guiIn.dst;
                pd4Controller.P = guiIn.pst;
                co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity)+2;
                logOut.state=24;
            }
              break;

        default:
            break;
    } // end switch

    // end control code //
}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCEqPoint)

}
}

// vim: expandtab
