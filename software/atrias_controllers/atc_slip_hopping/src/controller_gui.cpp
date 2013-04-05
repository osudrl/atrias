/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#include <atc_slip_hopping/controller_gui.h>

// guiInit =====================================================================
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {

	// Get widgets -------------------------------------------------------------
    //gui->get_widget("example_spinbutton", example_spinbutton);

	// Set ranges --------------------------------------------------------------
	//example_spinbutton->set_range(0.0, 1.0);
	
	// Set values --------------------------------------------------------------
	//example_spinbutton->set_value(0.0);
    
	// Set up subscriber and publisher.
    sub = nh.subscribe("controller_status", 0, controllerCallback);
    pub = nh.advertise<atc_slip_hopping::controller_input>("controller_input", 0);
    return true;

}


// controllerCallback ==========================================================
void controllerCallback(const atc_slip_hopping::controller_status &status) {

    controllerDataIn = status;
    
} // controllerCallback


// getParameters ===============================================================
void getParameters() {

    // Get parameters in the atrias_gui namespace ------------------------------
    //nh.getParam("/atrias_gui/example", controllerDataOut.example);

    // Configure the GUI -------------------------------------------------------
    //example_spinbutton->set_value(controllerDataOut.example);

} // getParameters


// setParameters ===============================================================
void setParameters() {
    //nh.setParam("/atrias_gui/example", controllerDataOut.example);
    
} // setParameters


// guiUpdate ===================================================================
void guiUpdate() {

	// Update GUI --------------------------------------------------------------
    //controllerDataOut.example = example_spinbutton->get_value();
    
    // Publish -----------------------------------------------------------------
    pub.publish(controllerDataOut);
    
} // guiUpdate

// guiTakedown =================================================================
void guiTakedown() {

} // guiTakedown

