/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the motor torque controller.
 */

#include <atc_motor_torque/controller_component.h>

namespace atrias {
namespace controller {

/**
 * Find A1/A2 ratio of sine wave.
 *
 * @param T Period in seconds
 * @param Tp Peak target current
 */
static float find_ratio_sine(float T, float Tp)
{
    static double a2integral = 0;
    for (int t=0; t<asin(AMC_IC/Tp); t+=0.001) {
        a2integral += (AMC_IC - Tp*sin(2*PI*t/T)) / 1000;
    }

    return (AMC_IP-AMC_IC)*(T/2*(PI-2*asin(AMC_IC/Tp)))/(AMC_IC*T/2+a2integral);
}

ATCMotorTorque::ATCMotorTorque(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCMotorTorque::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    addEventPort(guiDataIn);

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

    // Duty cycle test. This is temporary stuff.
    if (guiIn.dutyCycleTest) {
        if (guiIn.dc_mode == 0) {
            // Square wave
            if (dcCounter < guiIn.dc_tp*1000) {
                co.rLeg.motorCurrentA = guiIn.dc_ip;   // Apply peak current.
            }
            else if (dcCounter < (guiIn.dc_tp + guiIn.dc_tc)*1000) {
                co.rLeg.motorCurrentA = guiIn.dc_ic;   // Apply continuous current.
            }
            else {
                dcCounter = 0;
            }
        }
        else if (guiIn.dc_mode == 1) {
            // Sine wave
            co.rLeg.motorCurrentA = guiIn.dc_tp * sin(2*PI*dcCounter/1000*guiIn.dc_freq);
        }

        if (guiIn.dc_oscillate) {
            // Oscillate motor direction to keep applied current at maximum
            // (i.e., don't let the motor reach high velocity).
            co.rLeg.motorCurrentA *= ((int) (dcCounter * guiIn.dc_oscillate_freq / 1000)) % 2;

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
        log(Info) << "[ATCMT] fbC: " << fbCounter << "  curC: " << curCounter << "  cur lim: " << curLimit << "  A1/A2: " << find_ratio_sine(1/guiIn.dc_freq, ABS(co.rLeg.motorCurrentA)) << endlog();
    }

    // Output for RTOps
    return co;
}

// Don't put control code below here!

void ATCMotorTorque::estimateCurrentLimit(atrias_msgs::robot_state rs)
{
    double curTarget = co.rLeg.motorCurrentA;

    if ((rtOps::RtOpsState) rs.rtOpsState != rtOps::RtOpsState::ENABLED) {
        // Amps aren't in foldback if controller isn't enabled.
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
