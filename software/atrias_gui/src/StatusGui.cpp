/*
 * StatusGui.cpp
 *
 *  Created on: May 30, 2012
 *      Author: Michael Anderson
 */

#include <atrias_gui/StatusGui.h>

StatusGui::StatusGui(char *path) {
    // Create the relative path to the Glade file.
    std::string glade_gui_path = std::string(path);

    glade_gui_path = glade_gui_path.substr(0, glade_gui_path.rfind("/bin"));
    glade_gui_path.append("/media/status_gui.glade");

    gui = Gtk::Builder::create();
    try {
        gui->add_from_file(glade_gui_path);
    }
    catch (const Glib::FileError& ex) {
        ROS_ERROR("File Error");
        //          ROS_ERROR("FileError: %d", ex.what());
    }
    catch (const Gtk::BuilderError& ex) {
        ROS_ERROR("Builder Error");
        ROS_ERROR(ex.what().c_str());
        //          ROS_ERROR("BuilderError: %d", ex.what());
    }

    // Grab pointers to GUI objects
    gui->get_widget("status_window", status_window);
    if (!status_window) {
        ROS_ERROR("No Status Window");
    }


    // Boom
    gui->get_widget("azPosDisplay", azPosDisplay);
    gui->get_widget("elPosDisplay", elPosDisplay);
    gui->get_widget("azVelDisplay", azVelDisplay);
    gui->get_widget("elVelDisplay", elVelDisplay);

    // Torso
    gui->get_widget("xPosDisplay", xPosDisplay);
    gui->get_widget("yPosDisplay", yPosDisplay);
    gui->get_widget("zPosDisplay", zPosDisplay);
    gui->get_widget("xVelDisplay", xVelDisplay);
    gui->get_widget("yVelDisplay", yVelDisplay);
    gui->get_widget("zVelDisplay", zVelDisplay);
    
    // Legs
    gui->get_widget("leftLegLengthDisplay", leftLegLengthDisplay);
    gui->get_widget("leftLegAngleDisplay", leftLegAngleDisplay);
    gui->get_widget("rightLegLengthDisplay", rightLegLengthDisplay);
    gui->get_widget("rightLegAngleDisplay", rightLegAngleDisplay);
    gui->get_widget("leftHipAngleDisplay", leftHipAngleDisplay);
    gui->get_widget("rightHipAngleDisplay", rightHipAngleDisplay);

    // Springs
    gui->get_widget("spring_deflection_left_A_entry", spring_deflection_left_A_entry);
    gui->get_widget("spring_deflection_left_B_entry", spring_deflection_left_B_entry);
    gui->get_widget("spring_deflection_right_A_entry", spring_deflection_right_A_entry);
    gui->get_widget("spring_deflection_right_B_entry", spring_deflection_right_B_entry);

    // Motors
    gui->get_widget("torqueLeftADisplay", torqueLeftADisplay);
    gui->get_widget("torqueLeftBDisplay", torqueLeftBDisplay);
    gui->get_widget("torqueLeftHipDisplay", torqueLeftHipDisplay);
    gui->get_widget("torqueRightADisplay", torqueRightADisplay);
    gui->get_widget("torqueRightBDisplay", torqueRightBDisplay);
    gui->get_widget("torqueRightHipDisplay", torqueRightHipDisplay);
    gui->get_widget("motor_torqueLeftA_progress_bar", motor_torqueLeftA_progress_bar);
    gui->get_widget("motor_torqueLeftB_progress_bar", motor_torqueLeftB_progress_bar);
    gui->get_widget("motor_torqueLeftHip_progress_bar", motor_torqueLeftHip_progress_bar);
    gui->get_widget("motor_torqueRightA_progress_bar", motor_torqueRightA_progress_bar);
    gui->get_widget("motor_torqueRightB_progress_bar", motor_torqueRightB_progress_bar);
    gui->get_widget("motor_torqueRightHip_progress_bar", motor_torqueRightHip_progress_bar);

    // Toes
    gui->get_widget("leftToeDisplay", leftToeDisplay);
    gui->get_widget("rightToeDisplay", rightToeDisplay);

    // Medulla
    gui->get_widget("medullaLAError_entry", medullaLAError_entry);
    gui->get_widget("medullaLBError_entry", medullaLBError_entry);
    gui->get_widget("medullaRAError_entry", medullaRAError_entry);
    gui->get_widget("medullaRBError_entry", medullaRBError_entry);
    gui->get_widget("medullaLHipError_entry", medullaLHipError_entry);
    gui->get_widget("medullaRHipError_entry", medullaRHipError_entry);
    gui->get_widget("medullaBoomError_entry", medullaBoomError_entry);

    // Voltage
    gui->get_widget("leftALogicVoltage", leftALogicVoltage);
    gui->get_widget("leftBLogicVoltage", leftBLogicVoltage);
    gui->get_widget("rightALogicVoltage", rightALogicVoltage);
    gui->get_widget("rightBLogicVoltage", rightBLogicVoltage);
    gui->get_widget("leftAMotorVoltage", leftAMotorVoltage);
    gui->get_widget("leftBMotorVoltage", leftBMotorVoltage);
    gui->get_widget("rightAMotorVoltage", rightAMotorVoltage);
    gui->get_widget("rightBMotorVoltage", rightBMotorVoltage);

    usageIndex = 0;

    status_window->show_all();
}

StatusGui::~StatusGui() {

}

void StatusGui::update(rt_ops_cycle rtCycle) {
    update_robot_status(rtCycle);
}

void StatusGui::update_robot_status(rt_ops_cycle rtCycle) {
    char buffer[20];

    // Boom
    sprintf(buffer, "%0.4f Rad", rtCycle.robotState.position.xAngle);
    azPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f Rad", rtCycle.robotState.position.boomAngle);
    elPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f Rad/s", rtCycle.robotState.position.xAngleVelocity);
    azVelDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f Rad/s", rtCycle.robotState.position.boomAngleVelocity);
    elVelDisplay->set_text(buffer);

    // Torso
    sprintf(buffer, "%0.4f m", rtCycle.robotState.position.xPosition);
    xPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f m", rtCycle.robotState.position.yPosition);
    yPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f m", rtCycle.robotState.position.zPosition);
    zPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f m/s", rtCycle.robotState.position.xVelocity);
    xVelDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f m/s", rtCycle.robotState.position.yVelocity);
    yVelDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f m/s", rtCycle.robotState.position.zVelocity);
    zVelDisplay->set_text(buffer);

    // Legs
    sprintf(buffer, "%0.4f m", LEG_LENGTH(rtCycle.robotState.lLeg.halfA.legAngle, rtCycle.robotState.lLeg.halfB.legAngle));
    leftLegLengthDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f Rad", LEG_ANGLE(rtCycle.robotState.lLeg.halfA.legAngle, rtCycle.robotState.lLeg.halfB.legAngle));
    leftLegAngleDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f m", LEG_LENGTH(rtCycle.robotState.rLeg.halfA.legAngle, rtCycle.robotState.rLeg.halfB.legAngle));
    rightLegLengthDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f Rad", LEG_ANGLE(rtCycle.robotState.rLeg.halfA.legAngle, rtCycle.robotState.rLeg.halfB.legAngle));
    rightLegAngleDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f Rad", rtCycle.robotState.lLeg.hip.legBodyAngle);
    leftHipAngleDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f Rad", rtCycle.robotState.rLeg.hip.legBodyAngle);
    rightHipAngleDisplay->set_text(buffer);

    // Springs
    sprintf(buffer, "%0.4f Rad", rtCycle.robotState.lLeg.halfA.motorAngle - rtCycle.robotState.lLeg.halfA.legAngle);
    spring_deflection_left_A_entry->set_text(buffer);
    sprintf(buffer, "%0.4f Rad", rtCycle.robotState.lLeg.halfB.motorAngle - rtCycle.robotState.lLeg.halfB.legAngle);
    spring_deflection_left_B_entry->set_text(buffer);
    sprintf(buffer, "%0.4f Rad", rtCycle.robotState.rLeg.halfA.motorAngle - rtCycle.robotState.rLeg.halfA.legAngle);
    spring_deflection_right_A_entry->set_text(buffer);
    sprintf(buffer, "%0.4f Rad", rtCycle.robotState.rLeg.halfB.motorAngle - rtCycle.robotState.rLeg.halfB.legAngle);
    spring_deflection_right_B_entry->set_text(buffer);

    // Motors
    sprintf(buffer, "%0.4f A", rtCycle.commandedOutput.lLeg.motorCurrentA);
    torqueLeftADisplay->set_text(buffer);
    motor_torqueLeftA_progress_bar->set_fraction(MIN(ABS(rtCycle.commandedOutput.lLeg.motorCurrentA), MAX_MTR_CURRENT_CMD ) / MAX_MTR_CURRENT_CMD );
    sprintf(buffer, "%0.4f A", rtCycle.commandedOutput.lLeg.motorCurrentB);
    torqueLeftBDisplay->set_text(buffer);
    motor_torqueLeftB_progress_bar->set_fraction(MIN(ABS(rtCycle.commandedOutput.lLeg.motorCurrentB), MAX_MTR_CURRENT_CMD ) / MAX_MTR_CURRENT_CMD );
    sprintf(buffer, "%0.4f A", rtCycle.commandedOutput.lLeg.motorCurrentHip);
    torqueLeftHipDisplay->set_text(buffer);    
    motor_torqueLeftHip_progress_bar->set_fraction(MIN(ABS(rtCycle.commandedOutput.lLeg.motorCurrentHip), MAX_MTR_CURRENT_CMD ) / MAX_MTR_CURRENT_CMD );
    sprintf(buffer, "%0.4f A", rtCycle.commandedOutput.rLeg.motorCurrentA);
    torqueRightADisplay->set_text(buffer);
    motor_torqueRightA_progress_bar->set_fraction(MIN(ABS(rtCycle.commandedOutput.rLeg.motorCurrentA), MAX_MTR_CURRENT_CMD ) / MAX_MTR_CURRENT_CMD );
    sprintf(buffer, "%0.4f A", rtCycle.commandedOutput.rLeg.motorCurrentB);
    torqueRightBDisplay->set_text(buffer);
    motor_torqueRightB_progress_bar->set_fraction(MIN(ABS(rtCycle.commandedOutput.rLeg.motorCurrentB), MAX_MTR_CURRENT_CMD ) / MAX_MTR_CURRENT_CMD );
    sprintf(buffer, "%0.4f A", rtCycle.commandedOutput.rLeg.motorCurrentHip);
    torqueRightHipDisplay->set_text(buffer);
    motor_torqueRightHip_progress_bar->set_fraction(MIN(ABS(rtCycle.commandedOutput.rLeg.motorCurrentHip), MAX_MTR_CURRENT_CMD ) / MAX_MTR_CURRENT_CMD );

    // Toes
    sprintf(buffer, "%0.4u --> %0.1u", rtCycle.robotState.lLeg.toeSwitch, rtCycle.robotState.lLeg.onGround);
    leftToeDisplay->set_text(buffer);
    sprintf(buffer, "%0.4u --> %0.1u", rtCycle.robotState.rLeg.toeSwitch, rtCycle.robotState.rLeg.onGround);
    rightToeDisplay->set_text(buffer);

    // Medullas
    update_medulla_errors(rtCycle.robotState.lLeg.halfA.errorFlags,rtCycle.robotState.lLeg.halfA.limitSwitches,medullaLAError_entry);
    update_medulla_errors(rtCycle.robotState.lLeg.halfB.errorFlags,rtCycle.robotState.lLeg.halfB.limitSwitches,medullaLBError_entry);
    update_medulla_errors(rtCycle.robotState.rLeg.halfA.errorFlags,rtCycle.robotState.rLeg.halfA.limitSwitches,medullaRAError_entry);
    update_medulla_errors(rtCycle.robotState.rLeg.halfB.errorFlags,rtCycle.robotState.rLeg.halfB.limitSwitches,medullaRBError_entry);
    update_medulla_errors(rtCycle.robotState.lLeg.hip.errorFlags,rtCycle.robotState.lLeg.hip.limitSwitches,medullaLHipError_entry);
    update_medulla_errors(rtCycle.robotState.rLeg.hip.errorFlags,rtCycle.robotState.rLeg.hip.limitSwitches,medullaRHipError_entry);
    update_medulla_errors(rtCycle.robotState.boomMedullaErrorFlags,(uint8_t) 0, medullaBoomError_entry);

    // Voltage
    sprintf(buffer, "%0.4f V", rtCycle.robotState.lLeg.halfA.logicVoltage);
    leftALogicVoltage->set_text(buffer);
    sprintf(buffer, "%0.4f V", rtCycle.robotState.lLeg.halfB.logicVoltage);
    leftBLogicVoltage->set_text(buffer);
    sprintf(buffer, "%0.4f V", rtCycle.robotState.rLeg.halfA.logicVoltage);
    rightALogicVoltage->set_text(buffer);
    sprintf(buffer, "%0.4f V", rtCycle.robotState.rLeg.halfB.logicVoltage);
    rightBLogicVoltage->set_text(buffer);
    sprintf(buffer, "%0.4f V", rtCycle.robotState.lLeg.halfA.motorVoltage);
    leftAMotorVoltage->set_text(buffer);
    sprintf(buffer, "%0.4f V", rtCycle.robotState.lLeg.halfB.motorVoltage);
    leftBMotorVoltage->set_text(buffer);
    sprintf(buffer, "%0.4f V", rtCycle.robotState.rLeg.halfA.motorVoltage);
    rightAMotorVoltage->set_text(buffer);
    sprintf(buffer, "%0.4f V", rtCycle.robotState.rLeg.halfB.motorVoltage);
    rightBMotorVoltage->set_text(buffer);
    
    //TODO: Fix this
    /*if (usageIndex < CPU_USAGE_AVERAGE_TICKS) {
        usage[usageIndex] = rs.loopTime;
        usageIndex++;
    }
    else {
        uint32_t usageTotal = 0;
        for (size_t s = 0; s < CPU_USAGE_AVERAGE_TICKS; s++) {
            usageTotal += usage[s];
        }
        float fraction = (float)usageTotal / (float)CPU_USAGE_AVERAGE_TICKS  / CONTROL_LOOP_INTERVAL_USEC;
        sprintf(buffer, "%i%%", (int)(fraction * 100.));
        cpu_load_label->set_label(buffer);
        cpu_load_bar->set_fraction(fraction);

        usageIndex = 0;
    }*/
}

void StatusGui::update_medulla_errors(uint8_t errorFlags, uint8_t limitSwitches, Gtk::Entry *errorEntry)
{
    Glib::ustring error;
    // Medulla Error Flags
    if (errorFlags & medulla_error_estop)
        error += "EStop pressed, ";
    if (errorFlags & medulla_error_limit_switch)
        error += "Limit switch, ";
    if (errorFlags & medulla_error_thermistor)
        error += "Motor over temperature, ";
    if (errorFlags & medulla_error_motor_voltage)
        error += "Motor voltage low, ";
    if (errorFlags & medulla_error_logic_voltage)
        error += "Logic voltage low, ";
    if (errorFlags & medulla_error_encoder)
    	error += "Encoder error, ";
    if (errorFlags & medulla_error_halt)
        error += "Halt mode activated, ";
    if (errorFlags & medulla_error_amplifier)
        error += "Amplifier error, ";
    // Limit Switches
    if (limitSwitches & limit_switch_error_positive)
        error += "Positive limit switch, ";
    if (limitSwitches & limit_switch_error_negative)
        error += "Negative limit switch, ";
    if (limitSwitches & limit_switch_error_positive_deflection)
        error += "Positive deflection limit switch, ";
    if (limitSwitches & limit_switch_error_negative_deflection)
        error += "Negative deflection limit switch, ";
    if (limitSwitches & limit_switch_error_retraction)
        error += "Retraction limit switch, ";
    if (limitSwitches & limit_switch_error_extension)
        error += "Extension limit switch, ";

    errorEntry->set_text(error);
}
