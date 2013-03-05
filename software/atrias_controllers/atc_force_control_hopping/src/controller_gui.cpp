/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#include <atc_force_control_hopping/controller_gui.h>

// guiInit =====================================================================
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {

	// Get widgets

    
	// Set ranges

    
    // Set values


	// Set up subscriber and publisher.
    sub = nh.subscribe("atc_force_control_hopping_status", 0, controllerCallback);
    pub = nh.advertise<atc_force_control_hopping::controller_input>("atc_force_control_hopping_input", 0);
    return true;

} // guiInit

// controllerCallback ==========================================================
void controllerCallback(const atc_force_control_hopping::controller_status &status) {
    controllerDataIn = status;
    
} // controllerCallback

// getParameters ===============================================================
void getParameters() {
    // Get parameters in the atrias_gui namespace

    // Configure the GUI


} // getParameters

// setParameters ===============================================================
void setParameters() {

} // setParameters

// guiUpdate ===================================================================
void guiUpdate() {

    
    pub.publish(controllerDataOut);
    
} // guiUpdate

// guiTakedown =================================================================
void guiTakedown() {

} // guiTakedown

