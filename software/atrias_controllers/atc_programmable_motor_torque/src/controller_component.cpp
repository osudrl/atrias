/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the motor torque controller.
 */

#include <atc_programmable_motor_torque/controller_component.h>

namespace atrias {
namespace controller {

ATCProgrammableMotorTorque::ATCProgrammableMotorTorque(std::string name):
    RTT::TaskContext(name),
		guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCProgrammableMotorTorque::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    addEventPort(guiDataIn);

    currentSampleIndex = 0;

    log(Info) << "[ATCMT] Motor torque controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCProgrammableMotorTorque::runController(atrias_msgs::robot_state rs) {
    if ((uint8_t)rs.cmState == (uint8_t)controllerManager::RtOpsCommand::ENABLE) {
        if (currentSampleIndex < configData.size()) {
            if (fabs(guiIn.des_motor_torque_B) > CURRENT_ADJUSTMENT_THRESHOLD)
                controllerOutput.lLeg.motorCurrentB = configData[currentSampleIndex] + guiIn.des_motor_torque_B;
            else
                controllerOutput.lLeg.motorCurrentB = configData[currentSampleIndex];
            //printf("INDEX: %u\n", currentSampleIndex);
            currentSampleIndex++;
            return controllerOutput;
        }
    }
    else
        currentSampleIndex = 0;

    controllerOutput.lLeg.motorCurrentB = 0.;

    controllerOutput.command = medulla_state_run;

    // Output for RTOps
    return controllerOutput;
}

// Don't put control code below here!
bool ATCProgrammableMotorTorque::configureHook() {
    log(Info) << "[ATCMT] Loading config data!" << endlog();
    const char *homeDir = getenv("HOME");
    char path[128];
    strcpy(path, homeDir);
    strcat(path, "/torqueData.txt");
    FILE *configFile = fopen(path, "r");
    if (!configFile) {
        log(Info) << "[ATCMT] Failed to load config file!" << endlog();
        return false;
    }
    else {
        int c;
        char buf[32];
        uint8_t i = 0;
        double d;

        do {
            c = fgetc(configFile);
            if (c != EOF) {
                if (c != ',') {
                    buf[i] = c;
                    i++;
                }
                else {
                    buf[i] = '\0';
                    d = strtod(buf, NULL);
                    configData.push_back(d);
                    //printf("Input string: %s    Output double: %f", buf, d);
                    i = 0;
                }
            }
            else {
                buf[i] = '\0';
                d = strtod(buf, NULL);
                configData.push_back(d);
                //printf("Input string: %s    Output double: %f", buf, d);
            }
        } while (c != EOF);
    }
    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCProgrammableMotorTorque::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCProgrammableMotorTorque::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCProgrammableMotorTorque::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCProgrammableMotorTorque::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCProgrammableMotorTorque)

}
}
