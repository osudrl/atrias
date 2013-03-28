/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the motor torque controller.
 */

#include <atc_motor_torque/controller_component.h>

namespace atrias {
namespace controller {

ATCMotorTorque::ATCMotorTorque(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCMotorTorque::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // ASCSlipModel
    this->addProperty("ascSlipModel0Name", ascSlipModel0Name);

    addEventPort(guiDataIn);

    slipState.isFlight = false;

    dcCounter = 0;

    curLimit = AMC_IP;
    curCounter = COUNTER_MAX;
    fbCounter = AMC_PEAK_TIME + AMC_FOLDBACK_TIME;
    inFoldback = false;

    log(Info) << "[ATCMT] Motor torque controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCMotorTorque::runController(atrias_msgs::robot_state rs) {
    // Stuff the msg
    co.lLeg.motorCurrentA = guiIn.des_motor_torque_left_A;
    co.lLeg.motorCurrentB = guiIn.des_motor_torque_left_B;
    co.lLeg.motorCurrentHip = guiIn.des_motor_torque_left_hip;
    co.rLeg.motorCurrentA = guiIn.des_motor_torque_right_A;
    co.rLeg.motorCurrentB = guiIn.des_motor_torque_right_B;
    co.rLeg.motorCurrentHip = guiIn.des_motor_torque_right_hip;
    co.command = medulla_state_run;

    // Duty cycle test. This might get moved to a separate test controller.
    if (guiIn.dutyCycleTest) {
        // Are we in stance phase?
        if (guiIn.dc_mode == 0 || guiIn.dc_mode == 1) {
            // Use frequency and duty cycle inputs to determine phase.
            dcInStance = (dcCounter < 1000/guiIn.dc_freq * guiIn.dc_dc) ? true : false;
        }
        else if (guiIn.dc_mode == 2) {
            // Use slipState struct to determine phase.
            dcInStance = !slipState.isFlight;

            // Reset slipModel and slipState if at beginning of cycle.
            if (dcCounter == 0) {
                slipModel.g = -9.81;
                slipModel.k = guiIn.dc_spring_stiffness;
                slipModel.m = 607.5/9.81;
                slipModel.r0 = 0.85;
                slipState.r = 0.85;
                slipState.dr = -sqrt(2.8*9.81*guiIn.dc_hop_height);
                slipState.q = M_PI/2.0;
                slipState.dq = 0.0;
            }
        }
        else {
            dcInStance = false;
        }

        // Stance phase
        if (dcInStance) {
            // Square wave
            if (guiIn.dc_mode == 0) {
                co.rLeg.motorCurrentA = guiIn.dc_ip;   // Apply peak current.
            }
            // Half sine wave
            else if (guiIn.dc_mode == 1) {
                co.rLeg.motorCurrentA = guiIn.dc_ip * sin(PI/guiIn.dc_dc*dcCounter/1000.0*guiIn.dc_freq);
            }
            else if (guiIn.dc_mode == 2) {
                slipState = slipAdvance0(slipModel, slipState);
                LegForce legForce = slipForce0(slipModel, slipState);

               // Compute required joint torque using Jacobian.
                double legAngle = slipState.q - acos(slipState.r);
                double tauSpring = -legForce.fx * l2 * cos(legAngle + bodyPitch) + legForce.fz * l2 * sin(legAngle + bodyPitch);

                // Motor current
                co.rLeg.motorCurrentA = tauSpring/kg/kt;
            }
        }
        // Flight phase
        else {
            co.rLeg.motorCurrentA = guiIn.dc_ic;   // Apply continuous current.

            // End of flight phase
            if (guiIn.dc_mode == 0 || guiIn.dc_mode == 1) {
                if (dcCounter >= 1000/guiIn.dc_freq) {
                    dcCounter = 0;
                }
            }
            else if (guiIn.dc_mode == 2) {
                if (dcFlightEndTime == 0) {
                    dcFlightEndTime = dcCounter + 1000*sqrt(2.8*guiIn.dc_hop_height/9.81);
                }

                if (dcCounter >= dcFlightEndTime) {
                    dcCounter = 0;
                    dcFlightEndTime = 0;
                }
            }
        }

        // Oscillate motor direction to keep applied current at maximum (i.e.,
        // don't let the motor reach high velocity).
        if (guiIn.dc_oscillate) {
            co.rLeg.motorCurrentA *= (((int) (dcCounter * guiIn.dc_oscillate_freq / 500)) % 2) ? 1 : -1;
        }

        dcCounter++;
    }
    else {
        dcCounter = 0;
    }

    // Run current limit estimator.
    estimateCurrentLimit(rs);

    // Set current limit if GUI says so
    if (guiIn.limitCurrent) {
        if (co.rLeg.motorCurrentA > curLimit) {
            co.rLeg.motorCurrentA = curLimit;
        }
        else if (co.rLeg.motorCurrentA < -curLimit) {
            co.rLeg.motorCurrentA = -curLimit;
        }
    }

    static uint16_t n = 0;
    n = (n+1) % 100;

    if (n == 0) {
        log(Info) << "[ATCMT] fbC: " << fbCounter << "  curC: " << curCounter << "  cur lim: " << curLimit << endlog();
    }

    // Output for RTOps
    return co;
}

// Don't put control code below here!

void ATCMotorTorque::estimateCurrentLimit(atrias_msgs::robot_state rs)
{
    double curTarget = co.rLeg.motorCurrentA;

    // Amps aren't in foldback if controller isn't enabled.
    if ((rtOps::RtOpsState) rs.rtOpsState != rtOps::RtOpsState::ENABLED) {
        inFoldback = false;
        curTarget = 0.0;
    }
    else if (!inFoldback && ABS(curTarget) > AMC_IC) {
        inFoldback = true;

        // Reset counter to current limit.
        if (fbCounter < AMC_PEAK_TIME+AMC_FOLDBACK_TIME) {
            curCounter = curLimit;
        }
    }
    else if (inFoldback && ABS(curTarget) <= AMC_IC) {
        inFoldback = false;
    }

    // Update counters.
    if (inFoldback) {
        // Decrement counter at constant slope regardless of target current.
        if (curCounter > AMC_IC) {
            curCounter -= M_FB / 1000;
        }
        else {
            curCounter = AMC_IC;
        }

        // Decrement timer.
        if (fbCounter > 0) {
            fbCounter -= 0.001;
        }
    }
    else {
        // Increment counter based on target current.
        if (curCounter < COUNTER_MAX) {
            curCounter += M_FB * (AMC_IC-curTarget) / (AMC_IP-AMC_IC) / 2 / 1000;
        }
        else {
            curCounter = COUNTER_MAX;
        }

        // Increment timer.
        if (fbCounter < AMC_PEAK_TIME+AMC_FOLDBACK_TIME) {
            fbCounter += 0.001;
        }
    }

    // Set current limit to minimum among counter, recovery rate cap, and
    // peak current limit.
    curLimit = MIN(curCounter, MIN(AMC_IC+M_FB*fbCounter, AMC_IP));
}

bool ATCMotorTorque::configureHook() {
    // ASCSlipModel Service
    ascSlipModel0 = this->getPeer(ascSlipModel0Name);
    if (ascSlipModel0) {
        slipAdvance0 = ascSlipModel0->provides("ascSlipModel")->getOperation("slipAdvance");
        slipForce0 = ascSlipModel0->provides("ascSlipModel")->getOperation("slipForce");
    }

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCMotorTorque::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCMotorTorque::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCMotorTorque::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCMotorTorque::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCMotorTorque)

}
}
