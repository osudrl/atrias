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
        //			ROS_ERROR("FileError: %d", ex.what());
    }
    catch (const Gtk::BuilderError& ex) {
        ROS_ERROR("Builder Error");
        ROS_ERROR(ex.what().c_str());
        //			ROS_ERROR("BuilderError: %d", ex.what());
    }

    // Grab pointers to GUI objects
    gui->get_widget("status_window", status_window);
    if (!status_window) {
        ROS_ERROR("No Status Window");
    }

    gui->get_widget("motor_torqueA_progress_bar", motor_torqueA_progress_bar);
    gui->get_widget("motor_torqueB_progress_bar", motor_torqueB_progress_bar);
    gui->get_widget("motor_torqueHip_progress_bar", motor_torqueHip_progress_bar);

    gui->get_widget("xPosDisplay", xPosDisplay);
    gui->get_widget("yPosDisplay", yPosDisplay);
    gui->get_widget("zPosDisplay", zPosDisplay);
    gui->get_widget("xVelDisplay", xVelDisplay);
    gui->get_widget("yVelDisplay", yVelDisplay);
    gui->get_widget("zVelDisplay", zVelDisplay);

    gui->get_widget("torqueADisplay", torqueADisplay);
    gui->get_widget("torqueBDisplay", torqueBDisplay);
    gui->get_widget("torqueHipDisplay", torqueHipDisplay);

    gui->get_widget("legLengthDisplay", legLengthDisplay);
    gui->get_widget("legAngleDisplay", legAngleDisplay);

    gui->get_widget("spring_deflection_A_entry", spring_deflection_A_entry);
    gui->get_widget("spring_deflection_B_entry", spring_deflection_B_entry);
    gui->get_widget("spring_deflectionA_progress_bar", spring_deflectionA_progress_bar);
    gui->get_widget("spring_deflectionB_progress_bar", spring_deflectionB_progress_bar);

    gui->get_widget("velocityADisplay", velocityADisplay);
    gui->get_widget("velocityBDisplay", velocityBDisplay);
    gui->get_widget("velocityHipDisplay", velocityHipDisplay);

    gui->get_widget("motor_velocityA_progress_bar", motor_velocityA_progress_bar);
    gui->get_widget("motor_velocityB_progress_bar", motor_velocityB_progress_bar);
    gui->get_widget("motor_velocityHip_progress_bar", motor_velocityHip_progress_bar);

    /*
     * This block is for the Medulla Status section.
     */
    gui->get_widget("MedullaA_TempA", MedullaA_TempA);
    gui->get_widget("MedullaA_TempB", MedullaA_TempB);
    gui->get_widget("MedullaA_TempC", MedullaA_TempC);
    gui->get_widget("MedullaA_VLogic", MedullaA_VLogic);
    gui->get_widget("MedullaA_VMotor", MedullaA_VMotor);

    gui->get_widget("MedullaB_TempA", MedullaB_TempA);
    gui->get_widget("MedullaB_TempB", MedullaB_TempB);
    gui->get_widget("MedullaB_TempC", MedullaB_TempC);
    gui->get_widget("MedullaB_VLogic", MedullaB_VLogic);
    gui->get_widget("MedullaB_VMotor", MedullaB_VMotor);

    gui->get_widget("medullaAError_entry", medullaAError_entry);
    gui->get_widget("medullaBError_entry", medullaBError_entry);
    gui->get_widget("medullaHipError_entry", medullaHipError_entry);
    gui->get_widget("medullaBoomError_entry", medullaBoomError_entry);

    gui->get_widget("cpu_load_label", cpu_load_label);
    gui->get_widget("cpu_load_bar", cpu_load_bar);

    usageIndex = 0;

    status_window->show_all();
}

StatusGui::~StatusGui() {

}

void StatusGui::update(controller_status cs) {
	update_robot_status(cs);
	update_medulla_status(cs);
}

void StatusGui::update_robot_status(controller_status cs) {
	char buffer[20];

    // Update the motor torque progress bars and displays.
    sprintf(buffer, "%0.4f", cs.state.motor_torqueA);
    torqueADisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", cs.state.motor_torqueB);
    torqueBDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", cs.state.motor_torque_hip);
    torqueHipDisplay->set_text(buffer);

    motor_torqueA_progress_bar->set_fraction(MIN(ABS(cs.state.motor_torqueA), MTR_MAX_TRQ) / MTR_MAX_TRQ);
    motor_torqueB_progress_bar->set_fraction(MIN(ABS(cs.state.motor_torqueB), MTR_MAX_TRQ) / MTR_MAX_TRQ);
    motor_torqueHip_progress_bar->set_fraction(MIN(ABS(cs.state.motor_torque_hip), MTR_MAX_TRQ) / MTR_MAX_TRQ);

    motor_velocityA_progress_bar->set_fraction(MIN(ABS(cs.state.motor_velocityA * 477.45), 1327) / 1327);
    motor_velocityB_progress_bar->set_fraction(MIN(ABS(cs.state.motor_velocityB * 477.45), 1327) / 1327);
    motor_velocityHip_progress_bar->set_fraction(MIN(ABS(cs.state.motor_velocity_hip), 10) / 10);
    sprintf(buffer, "%0.2f", cs.state.motor_velocityA * 477.45);
    velocityADisplay->set_text(buffer);
    sprintf(buffer, "%0.2f", cs.state.motor_velocityB * 477.45);
    velocityBDisplay->set_text(buffer);
    sprintf(buffer, "%0.2f", cs.state.motor_velocity_hip);
    velocityHipDisplay->set_text(buffer);

    // Update spring deflection displays.
    sprintf(buffer, "%0.8f", cs.state.motor_angleA - cs.state.leg_angleA);
    spring_deflection_A_entry->set_text(buffer);
    sprintf(buffer, "%0.8f", cs.state.motor_angleB - cs.state.leg_angleB);
    spring_deflection_B_entry->set_text(buffer);

    //replace with a macro?
    spring_deflectionA_progress_bar->set_fraction(
            log10(abs(cs.state.motor_angleA - cs.state.leg_angleA) + 1)
            / log10(21));

    spring_deflectionB_progress_bar->set_fraction(
            log10(abs(cs.state.motor_angleA - cs.state.leg_angleA) + 1)
            / log10(21));

    // Update the boom stuff indicatocs.state.
    sprintf(buffer, "%0.4f", cs.state.xPosition);
    xPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", cs.state.yPosition);
    yPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", cs.state.zPosition);
    zPosDisplay->set_text(buffer);

    sprintf(buffer, "%0.4f", cs.state.xVelocity);
    xVelDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", cs.state.yVelocity);
    yVelDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", cs.state.zVelocity);
    zVelDisplay->set_text(buffer);

    sprintf(buffer, "%0.4f", LEG_LENGTH(cs.state.motor_angleA, cs.state.motor_angleB));
    legLengthDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", LEG_ANGLE(cs.state.motor_angleA, cs.state.motor_angleB));
    legAngleDisplay->set_text(buffer);

    if (usageIndex < CPU_USAGE_AVERAGE_TICKS) {
        usage[usageIndex] = cs.state.loopTime;
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
    }
}

void StatusGui::update_medulla_status(controller_status cs) {
    char buffer[20];

    sprintf(buffer,"%0.2f",cs.state.thermistorA[0]);
    MedullaA_TempA->set_text(buffer);

    sprintf(buffer,"%0.2f",cs.state.thermistorA[1]);
    MedullaA_TempB->set_text(buffer);

    sprintf(buffer,"%0.2f",cs.state.thermistorA[2]);
    MedullaA_TempC->set_text(buffer);

    sprintf(buffer,"%0.1f",cs.state.motorVoltageA);
    MedullaA_VMotor->set_text(buffer);

    sprintf(buffer,"%0.1f",cs.state.logicVoltageA);
    MedullaA_VLogic->set_text(buffer);

    sprintf(buffer,"%0.2f",cs.state.thermistorB[0]);
    MedullaB_TempA->set_text(buffer);

    sprintf(buffer,"%0.2f",cs.state.thermistorB[1]);
    MedullaB_TempB->set_text(buffer);

    sprintf(buffer,"%0.2f",cs.state.thermistorB[2]);
    MedullaB_TempC->set_text(buffer);

    sprintf(buffer,"%0.1f",cs.state.motorVoltageB);
    MedullaB_VMotor->set_text(buffer);

    sprintf(buffer,"%0.1f",cs.state.logicVoltageB);
    MedullaB_VLogic->set_text(buffer);

    Glib::ustring error;
    if (cs.state.medullaStatusA & STATUS_ESTOP)
        error += "EStop Pressed, ";
    if (cs.state.medullaStatusA & STATUS_LIMITSW)
        error += "Limit Switch, ";
    if (cs.state.medullaStatusA & STATUS_OVER_TEMP)
        error += "Motor Over Temperature, ";
    if (cs.state.medullaStatusA & STATUS_MOTOR_OUT_OF_RANGE)
        error += "Motor Out of Range, ";
    if (cs.state.medullaStatusA & STATUS_MOTOR_CTRL_DISABLE)
        error += "Motor Control Disabled, ";
    if (cs.state.medullaStatusA & STATUS_MOTOR_VOLTAGE_LOW)
        error += "Motor Voltage Low, ";
    if (cs.state.medullaStatusA & STATUS_LOGIC_VOLTAGE_LOW)
        error += "Logic Voltage Low, ";
    if (cs.state.medullaStatusA & STATUS_ENCODER_ERROR)
            error += "Encoder Error ";

    medullaAError_entry->set_text(error);

    error = "";
    if (cs.state.medullaStatusB & STATUS_ESTOP)
        error += "EStop Pressed, ";
    if (cs.state.medullaStatusB & STATUS_LIMITSW)
        error += "Limit Switch, ";
    if (cs.state.medullaStatusB & STATUS_OVER_TEMP)
        error += "Motor Over Temperature, ";
    if (cs.state.medullaStatusB & STATUS_MOTOR_OUT_OF_RANGE)
        error += "Motor Out of Range, ";
    if (cs.state.medullaStatusB & STATUS_MOTOR_CTRL_DISABLE)
        error += "Motor Control Disabled, ";
    if (cs.state.medullaStatusB & STATUS_MOTOR_VOLTAGE_LOW)
        error += "Motor Voltage Low ";
    if (cs.state.medullaStatusB & STATUS_LOGIC_VOLTAGE_LOW)
        error += "Logic Voltage Low ";
    if (cs.state.medullaStatusB & STATUS_ENCODER_ERROR)
        error += "Encoder Error ";

    medullaBError_entry->set_text(error);

    error = "";
    if (cs.state.medullaStatusHip & STATUS_ESTOP)
        error += "EStop Pressed, ";
    if (cs.state.medullaStatusHip & STATUS_LIMITSW)
        error += "Limit Switch, ";
    if (cs.state.medullaStatusHip & STATUS_OVER_TEMP)
        error += "Motor Over Temperature, ";
    if (cs.state.medullaStatusHip & STATUS_MOTOR_OUT_OF_RANGE)
        error += "Motor Out of Range, ";
    if (cs.state.medullaStatusHip & STATUS_MOTOR_CTRL_DISABLE)
        error += "Motor Control Disabled, ";
    if (cs.state.medullaStatusHip & STATUS_MOTOR_VOLTAGE_LOW)
        error += "Motor Voltage Low ";
    if (cs.state.medullaStatusHip & STATUS_LOGIC_VOLTAGE_LOW)
        error += "Logic Voltage Low ";
    if (cs.state.medullaStatusHip & STATUS_ENCODER_ERROR)
        error += "Encoder Error ";

    medullaHipError_entry->set_text(error);

    error = "";
    if (cs.state.medullaStatusBoom & STATUS_ESTOP)
        error += "EStop Pressed, ";
    if (cs.state.medullaStatusBoom & STATUS_LIMITSW)
        error += "Limit Switch, ";
    if (cs.state.medullaStatusBoom & STATUS_OVER_TEMP)
        error += "Motor Over Temperature, ";
    if (cs.state.medullaStatusBoom & STATUS_MOTOR_OUT_OF_RANGE)
        error += "Motor Out of Range, ";
    if (cs.state.medullaStatusBoom & STATUS_MOTOR_CTRL_DISABLE)
        error += "Motor Control Disabled, ";
    if (cs.state.medullaStatusBoom & STATUS_MOTOR_VOLTAGE_LOW)
        error += "Motor Voltage Low ";
    if (cs.state.medullaStatusBoom & STATUS_LOGIC_VOLTAGE_LOW)
        error += "Logic Voltage Low ";
    if (cs.state.medullaStatusBoom & STATUS_ENCODER_ERROR)
        error += "Encoder Error ";

    medullaBoomError_entry->set_text(error);
}
