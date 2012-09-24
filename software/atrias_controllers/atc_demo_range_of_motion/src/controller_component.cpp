/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_demo_range_of_motion controller.
 */

#include <atc_demo_range_of_motion/controller_component.h>

namespace atrias {
namespace controller {

ATCDemoRangeOfMotion::ATCDemoRangeOfMotion(std::string name):
    RTT::TaskContext(name),
    autoDemoStep(0),
    logPort(name + "_log"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCDemoRangeOfMotion::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties
    this->addProperty("pd0Name", pd0Name);
    this->addProperty("pd1Name", pd1Name);
    this->addProperty("pd2Name", pd2Name);
    this->addProperty("pd3Name", pd3Name);
    this->addProperty("pd4Name", pd4Name);
    this->addProperty("pd5Name", pd5Name);
    this->addProperty("spg0Name", spg0Name);
    this->addProperty("spg1Name", spg1Name);
    this->addProperty("spg2Name", spg2Name);
    this->addProperty("spg3Name", spg3Name);
    this->addProperty("spg4Name", spg4Name);
    this->addProperty("spg5Name", spg5Name);

    // For the GUI
    addEventPort(guiDataIn);
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



atrias_msgs::controller_output ATCDemoRangeOfMotion::runController(atrias_msgs::robot_state rs) {
    // Do nothing unless told otherwise
    co.lLeg.motorCurrentA   = 0.0;
    co.lLeg.motorCurrentB   = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA   = 0.0;
    co.rLeg.motorCurrentB   = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
        return co;

    // begin control code //

    // Set gains
    // Left leg motor A
    P0.set(guiIn.leg_p_gain);
    D0.set(guiIn.leg_d_gain);
    // Left leg motor B
    P1.set(guiIn.leg_p_gain);
    D1.set(guiIn.leg_d_gain);
    // Left leg motor hip
    P2.set(guiIn.hip_p_gain);
    D2.set(guiIn.hip_d_gain);
    // Right leg motor A
    P3.set(guiIn.leg_p_gain);
    D3.set(guiIn.leg_d_gain);
    // Right leg motor B
    P4.set(guiIn.leg_p_gain);
    D4.set(guiIn.leg_d_gain);
    // Right leg motor hip
    P5.set(guiIn.hip_p_gain);
    D5.set(guiIn.hip_d_gain);

    // If GUI input differs from current desired motor positions, update
    // current desired motor positions and reinitialize the smooth path
    // generators.
    if (guiIn.mode == 0 &&
            spg0IsFinished &&
            spg1IsFinished &&
            spg2IsFinished &&
            spg3IsFinished &&
            spg4IsFinished &&
            spg5IsFinished) {   // Manual mode.
        autoDemoStep = 0;
        if (desLeftAState.ang != guiIn.desLeftAPos && spg0IsFinished.get()) {
            desLeftAState.ang = guiIn.desLeftAPos;
            spg0Init(rs.lLeg.halfA.motorAngle, desLeftAState.ang, guiIn.legDuration);
        }
        if (desLeftBState.ang != guiIn.desLeftBPos && spg1IsFinished.get()) {
            desLeftBState.ang = guiIn.desLeftBPos;
            spg1Init(rs.lLeg.halfB.motorAngle, desLeftBState.ang, guiIn.legDuration);
        }
        if (desLeftHipState.ang != guiIn.desLeftHipPos && spg2IsFinished.get()) {
            desLeftHipState.ang = guiIn.desLeftHipPos;
            spg2Init(rs.lLeg.hip.legBodyAngle, desLeftHipState.ang, guiIn.hipDuration);
        }
        if (desRightAState.ang != guiIn.desRightAPos && spg3IsFinished.get()) {
            desRightAState.ang = guiIn.desRightAPos;
            spg3Init(rs.rLeg.halfA.motorAngle, desRightAState.ang, guiIn.legDuration);
        }
        if (desRightBState.ang != guiIn.desRightBPos && spg4IsFinished.get()) {
            desRightBState.ang = guiIn.desRightBPos;
            spg4Init(rs.rLeg.halfB.motorAngle, desRightBState.ang, guiIn.legDuration);
        }
        if (desRightHipState.ang != guiIn.desRightHipPos && spg5IsFinished.get()) {
            desRightHipState.ang = guiIn.desRightHipPos;
            spg5Init(rs.rLeg.hip.legBodyAngle, desRightHipState.ang, guiIn.hipDuration);
        }
    }
    else if (guiIn.mode == 1) {   // Few-at-a-time mode.
        static double la, lb, lh, ra, rb, rh;   // For temporary storage of angles for easier typing of the math.
        switch (autoDemoStep) {
            case 0:
                // Move Left A all the way up and Left B far away from A.
                if (spg0IsFinished && spg1IsFinished && spg3IsFinished && spg4IsFinished) {
                    la = LEG_A_MOTOR_MIN_LOC + (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    lb = la + (LEG_LOC_DIFF_MAX - LEG_LOC_SAFETY_DISTANCE - 0.1);
                    rb = LEG_B_MOTOR_MAX_LOC - (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    ra = rb - (LEG_LOC_DIFF_MAX - LEG_LOC_SAFETY_DISTANCE - 0.1);
                    spg0Init(rs.lLeg.halfA.motorAngle, la, guiIn.legDuration);
                    spg1Init(rs.lLeg.halfB.motorAngle, lb, guiIn.legDuration);
                    spg3Init(rs.rLeg.halfA.motorAngle, ra, guiIn.legDuration);
                    spg4Init(rs.rLeg.halfB.motorAngle, rb, guiIn.legDuration);
                    autoDemoStep++;
                }
                break;
            case 1: 
                // Move Left B close to Left A.
                if (spg0IsFinished && spg1IsFinished && spg3IsFinished && spg4IsFinished) {
                    lb = LEG_B_MOTOR_MIN_LOC + (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    ra = LEG_A_MOTOR_MAX_LOC - (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    spg1Init(rs.lLeg.halfB.motorAngle, lb, guiIn.legDuration);
                    spg4Init(rs.rLeg.halfB.motorAngle, rb, guiIn.legDuration);
                    autoDemoStep++;
                }
                break;
            case 2:
                // Move Left B all the way up and keep A close enough to avoid hitting hardstop.
                if (spg1IsFinished && spg4IsFinished) {
                    lb = LEG_B_MOTOR_MAX_LOC - (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    la = LEG_A_MOTOR_MAX_LOC - (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    ra = LEG_A_MOTOR_MIN_LOC + (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    rb = LEG_B_MOTOR_MIN_LOC + (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    spg1Init(rs.lLeg.halfB.motorAngle, lb, guiIn.legDuration);
                    spg0Init(rs.lLeg.halfA.motorAngle, la, guiIn.legDuration);
                    spg4Init(rs.rLeg.halfA.motorAngle, ra, guiIn.legDuration);
                    spg3Init(rs.rLeg.halfB.motorAngle, rb, guiIn.legDuration);
                    autoDemoStep++;
                }
                break;
            case 3:
                // Move Left A close to Left B.
                if (spg0IsFinished && spg1IsFinished && spg3IsFinished && spg4IsFinished) {
                    la = lb - (LEG_LOC_DIFF_MAX - LEG_LOC_SAFETY_DISTANCE - 0.1);
                    rb = ra + (LEG_LOC_DIFF_MAX - LEG_LOC_SAFETY_DISTANCE - 0.1);
                    spg0Init(rs.lLeg.halfA.motorAngle, la, guiIn.legDuration);
                    spg3Init(rs.rLeg.halfA.motorAngle, ra, guiIn.legDuration);
                    autoDemoStep++;
                }
                break;
            case 4:
                // Back to neutral with toe pointing down.
                if (spg0IsFinished && spg3IsFinished) {
                    la = 0.5*M_PI - (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    lb = 0.5*M_PI + (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    rb = 0.5*M_PI + (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    ra = 0.5*M_PI - (LEG_LOC_SAFETY_DISTANCE + 0.1);
                    spg0Init(rs.lLeg.halfA.motorAngle, la, guiIn.legDuration);
                    spg1Init(rs.lLeg.halfB.motorAngle, lb, guiIn.legDuration);
                    spg3Init(rs.rLeg.halfA.motorAngle, ra, guiIn.legDuration);
                    spg4Init(rs.rLeg.halfB.motorAngle, rb, guiIn.legDuration);
                    autoDemoStep = 0;
                }
                break;
        }
    }

    // Run the smooth path generators.
    desLeftAState    = spg0RunController();
    desLeftBState    = spg1RunController();
    desLeftHipState  = spg2RunController();
    desRightAState   = spg3RunController();
    desRightBState   = spg4RunController();
    desRightHipState = spg5RunController();

    // Stuff the msg
    co.lLeg.motorCurrentA   = pd0RunController(desLeftAState.ang,    rs.lLeg.halfA.motorAngle, desLeftAState.vel,    rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB   = pd1RunController(desLeftBState.ang,    rs.lLeg.halfB.motorAngle, desLeftBState.vel,    rs.lLeg.halfB.motorVelocity);
    co.lLeg.motorCurrentHip = pd2RunController(desLeftHipState.ang,  rs.lLeg.hip.legBodyAngle, desLeftHipState.vel,  rs.lLeg.hip.motorVelocity);
    co.rLeg.motorCurrentA   = pd0RunController(desRightAState.ang,   rs.rLeg.halfA.motorAngle, desRightAState.vel,   rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB   = pd1RunController(desRightBState.ang,   rs.rLeg.halfB.motorAngle, desRightBState.vel,   rs.rLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentHip = pd2RunController(desRightHipState.ang, rs.rLeg.hip.legBodyAngle, desRightHipState.vel, rs.rLeg.hip.motorVelocity);

    // end control code //

    // Command a run state
    co.command = medulla_state_run;

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCDemoRangeOfMotion::configureHook() {
    // Connect to the subcontrollers
    pd0 = this->getPeer(pd0Name);
    if (pd0) {
        pd0RunController = pd0->provides("pd")->getOperation("runController");
        P0 = pd0->properties()->getProperty("P");
        D0 = pd0->properties()->getProperty("D");
    }

    pd1 = this->getPeer(pd1Name);
    if (pd1) {
        pd1RunController = pd1->provides("pd")->getOperation("runController");
        P1 = pd1->properties()->getProperty("P");
        D1 = pd1->properties()->getProperty("D");
    }

    pd2 = this->getPeer(pd2Name);
    if (pd2) {
        pd2RunController = pd2->provides("pd")->getOperation("runController");
        P2 = pd2->properties()->getProperty("P");
        D2 = pd2->properties()->getProperty("D");
    }

    pd3 = this->getPeer(pd3Name);
    if (pd3) {
        pd3RunController = pd3->provides("pd")->getOperation("runController");
        P3 = pd3->properties()->getProperty("P");
        D3 = pd3->properties()->getProperty("D");
    }

    pd4 = this->getPeer(pd4Name);
    if (pd4) {
        pd4RunController = pd4->provides("pd")->getOperation("runController");
        P4 = pd4->properties()->getProperty("P");
        D4 = pd4->properties()->getProperty("D");
    }

    pd5 = this->getPeer(pd5Name);
    if (pd5) {
        pd5RunController = pd5->provides("pd")->getOperation("runController");
        P5 = pd5->properties()->getProperty("P");
        D5 = pd5->properties()->getProperty("D");
    }

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

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCDemoRangeOfMotion::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCDemoRangeOfMotion::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCDemoRangeOfMotion::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCDemoRangeOfMotion::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCDemoRangeOfMotion)

}
}
