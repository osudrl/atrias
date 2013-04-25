#include "atc_slip_walking/ATCSlipWalking.hpp"

namespace atrias {
namespace controller {

ATCSlipWalking::ATCSlipWalking(string name) :
    ATC(name),
    commonToolkit(this, "commonToolkit"),
    pdHip(this, "pdHip"),
    pdStanceLeg(this, "pdStanceLeg"),
    pdFlightLeg(this, "pdFlightLeg"),
    hipBoomKinematics(this, "hipBoomKinematics")
{
    // Initially ramp up the current over several seconds
    setStartupEnabled(true);

    hipControlSetup();
    eqPointWalkingSetup();
}

void ATCSlipWalking::controller() {
    // Robot state == rs
    // Controller output == co

    // Do nothing if disabled
    if (!isEnabled())
        return;

    // Setup
    guiCommunication();
    hipControl();

    // Main state machine
    switch (guiIn.main_controller) {
        case 0:
            standingControl();
            break;

        case 1:
            walkingControl();
            break;

        default:
            shutdownControl();
    }

    // Logging
    logOut.dummy = 0.0;
}

void ATCSlipWalking::guiCommunication() {
    // Input angles
    qTO = guiIn.takeoff_angle;
    qTD = guiIn.touchdown_angle;
    lsl = guiIn.stance_leg_length;
    // PD Gains
    pdStanceLeg.P = guiIn.leg_stance_kp;
    pdStanceLeg.D = guiIn.leg_stance_kd;
    pdFlightLeg.P = guiIn.leg_flight_kp;
    pdFlightLeg.D = guiIn.leg_flight_kd;
    pdHip.P       = guiIn.hip_kp;
    pdHip.D       = guiIn.hip_kd;

    // Copy over positions to the GUI output data
    guiOut.isEnabled = isEnabled();
}

void ATCSlipWalking::hipControlSetup() {
    // Circle radii around the boom
    toePosition.left = 2.15;
    toePosition.right = 2.45;
}


void ATCSlipWalking::hipControl() {
    // Hold the hips so the legs walk along a circle
    std::tie(qlh, qrh) = hipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
    co.lLeg.motorCurrentHip = pdHip(qlh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdHip(qrh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
}


void ATCSlipWalking::standingControl() {
    // Leg angles
    qrl = qTD;
    qll = qTO;
    // Leg lengths
    lrl = lll = lsl;

    // Compute motor angles
    std::tie(qlmA, qlmB) = commonToolkit.legPos2MotorPos(qll, lll);
    std::tie(qrmA, qrmB) = commonToolkit.legPos2MotorPos(qrl, lrl);

    co.lLeg.motorCurrentA = pdStanceLeg(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB = pdStanceLeg(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentA = pdStanceLeg(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB = pdStanceLeg(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


void ATCSlipWalking::walkingControl() {
    switch (guiIn.walking_controller) {
        case 0:
            eqPointWalkingControl(); // To get moving
            break;

        case 1:
            slipWalking();
            break;

        default: // This shouldn't happen
            shutdownControl();
    }
}

void ATCSlipWalking::eqPointWalkingSetup() {
    std::cout << "Setup: Right leg in stance" << std::endl;
    rsStanceLeg = &rs.rLeg;
    coStanceLeg = &co.rLeg;
    rsFlightLeg = &rs.lLeg;
    coFlightLeg = &co.lLeg;
    sw_stance = false;
    sw_flight = false;
    t=0.0;
}

// Control concepts from atc_eq_point
void ATCSlipWalking::eqPointWalkingControl() {
    switch (guiIn.ground_contact_method) {
        case 0: // Manual
            rGC = guiIn.right_ground_contact;
            lGC = guiIn.left_ground_contact;
            break;
        default: // Don't use ground contact
            rGC = true;
            lGC = true;
    }

    // Switching conditions
    if ((t>0.99) && rGC) {
        std::cout << "Right leg in stance" << std::endl;
        rsStanceLeg = &rs.rLeg;
        coStanceLeg = &co.rLeg;
        rsFlightLeg = &rs.lLeg;
        coFlightLeg = &co.lLeg;
        sw_stance = false;
        sw_flight = false;
        t=0.0;
    } else if ((t>0.99) && lGC) {
        std::cout << "Left leg in stance" << std::endl;
        rsStanceLeg = &rs.lLeg;
        coStanceLeg = &co.lLeg;
        rsFlightLeg = &rs.rLeg;
        coFlightLeg = &co.rLeg;
        sw_stance = false;
        sw_flight = false;
        t=0.0;
    }

    // Virtual leg angles
    qsl = (rsStanceLeg->halfA.motorAngle+rsStanceLeg->halfB.motorAngle)/2.0;
    qfl = (rsFlightLeg->halfA.motorAngle+rsFlightLeg->halfB.motorAngle)/2.0;

    // Map stance from touchdown to takeoff (from 0 to 1)
    s = (qsl-qTD) / (qTO-qTD);
    // Ratchet t along with s
    if (s > t)
        t = s;

    // Desired position for stance motor B
    qsmB_des = qTO + acos(lsl);
    // If the stance leg needs to be rotated
    if ((rsStanceLeg->halfB.motorAngle < qsmB_des) && !sw_stance) {
        // Stance motor A and B angles
        std::tie(qsmA, qsmB) = commonToolkit.legPos2MotorPos(qsl,lsl);
        // Advance position
        coStanceLeg->motorCurrentA = pdStanceLeg(qsmA, rsStanceLeg->halfA.motorAngle,0.5,rsStanceLeg->halfA.motorVelocity);
        coStanceLeg->motorCurrentB = pdStanceLeg(qsmB, rsStanceLeg->halfB.motorAngle,0.5,rsStanceLeg->halfB.motorVelocity) + guiIn.stance_current_offset;

    // The stance leg is fully extended
    } else {
        sw_stance = true;
        // Stance motor A and B angles
        std::tie(qsmA, qsmB) = commonToolkit.legPos2MotorPos(qTO,lsl);
        // Hold position
        coStanceLeg->motorCurrentA = pdStanceLeg(qsmA, rsStanceLeg->halfA.motorAngle,0.0,rsStanceLeg->halfA.motorVelocity);
        coStanceLeg->motorCurrentB = pdStanceLeg(qsmB, rsStanceLeg->halfB.motorAngle,0.0,rsStanceLeg->halfB.motorVelocity);
    }

    // If the flight leg needs to be rotated
    if ((t < 1.0) && (!sw_flight)) {
        // Amplitude between stance and flight leg lengths
        amp = lsl - guiIn.min_flight_leg_length;
        // Takeoff
        if (t < 0.1) {
            // Length and angle of flight leg
            lfl = lsl - amp * sin(t/guiIn.t_swing*M_PI);
            qfl = qTO;
        // Swing forward
        } else if (t < guiIn.t_swing) {
            lfl = lsl - amp * sin(t/guiIn.t_swing*M_PI);
            qfl = qTO - (qTO-qTD) * (t-0.1) / (guiIn.t_swing-0.1) * (1+guiIn.t_extension);
        // Retract
        } else {
            lfl = lsl;
            qfl = qTD - (qTO-qTD) * (1.0-t) / (1.0-guiIn.t_swing) * guiIn.t_extension;
        }
        // Flight motor A and B angles
        std::tie(qfmA, qfmB) = commonToolkit.legPos2MotorPos(qfl,lfl);
        coFlightLeg->motorCurrentA = pdFlightLeg(qfmA,rsFlightLeg->halfA.motorAngle,0.0,rsFlightLeg->halfA.motorVelocity);
        coFlightLeg->motorCurrentB = pdFlightLeg(qfmB,rsFlightLeg->halfB.motorAngle,0.0,rsFlightLeg->halfB.motorVelocity);

    // The other leg finished stance, so this leg is now in stance
    } else {
        sw_flight=true;
        std::tie(qfmA, qfmB) = commonToolkit.legPos2MotorPos(qTD,lsl);
        coFlightLeg->motorCurrentA = pdStanceLeg(qfmA,rsFlightLeg->halfA.motorAngle,0.0,rsFlightLeg->halfA.motorVelocity);
        coFlightLeg->motorCurrentB = pdStanceLeg(qfmB,rsFlightLeg->halfB.motorAngle,0.0,rsFlightLeg->halfB.motorVelocity);
    }

} // eqPointWalking

void ATCSlipWalking::slipWalking() {
    // Uses the control concept from atc_slip_hopping
    // TODO: Implement this

    shutdownControl();

}

void ATCSlipWalking::shutdownControl() {
    // Damping
    co.lLeg.motorCurrentHip = pdHip(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdHip(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);
    co.lLeg.motorCurrentA   = pdStanceLeg(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB   = pdStanceLeg(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentA   = pdStanceLeg(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB   = pdStanceLeg(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
}


// The OROCOS component object
ORO_CREATE_COMPONENT(ATCSlipWalking)

}
}

// vim: expandtab
