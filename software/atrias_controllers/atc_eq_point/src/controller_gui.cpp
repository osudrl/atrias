/*
 * controller_gui.cpp
 *
 * atc_eq_point controller
 *
 *  Created on: May 5, 2012
 *	  Author: Michael Anderson
 */

#include <atc_eq_point/controller_gui.h>

void gc_l_pressed()  {controllerDataOut.gc_l = 1;}
void gc_l_released() {controllerDataOut.gc_l = 0;}
void gc_r_pressed()  {controllerDataOut.gc_r = 1;}
void gc_r_released() {controllerDataOut.gc_r = 0;}

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	gui->get_widget("control_combobox", control_combobox);
	gui->get_widget("gc_l_button",      gc_l_button);
	gui->get_widget("gc_r_button",      gc_r_button);
	gui->get_widget("aea_spinbutton",   aea_spinbutton);
	gui->get_widget("pea_spinbutton",   pea_spinbutton);
	gui->get_widget("leftHipPos",       lhip_pos_spinbutton);
	gui->get_widget("rightHipPos",      rhip_pos_spinbutton);
	gui->get_widget("lst",              lst);
	gui->get_widget("lfl",              lfl);
	gui->get_widget("pst",              pst);
	gui->get_widget("pfl1",             pfl1);
	gui->get_widget("pfl2",             pfl2);
	gui->get_widget("dst",              dst);
	gui->get_widget("dfl1",             dfl1);
	gui->get_widget("dfl2",             dfl2);
	gui->get_widget("tsw",              tsw);
	gui->get_widget("aover",            aover);
	gui->get_widget("loc",              loc);
	gui->get_widget("rco",              rco);
	gui->get_widget("thip",             thip);
	gui->get_widget("tab",              tab);

	if (CHECK_WIDGET(control_combobox)    ||
	    CHECK_WIDGET(gc_l_button)         ||
	    CHECK_WIDGET(gc_r_button)         ||
	    CHECK_WIDGET(aea_spinbutton)      ||
	    CHECK_WIDGET(pea_spinbutton)      ||
	    CHECK_WIDGET(lhip_pos_spinbutton) ||
	    CHECK_WIDGET(rhip_pos_spinbutton) ||
	    CHECK_WIDGET(lst)                 ||
	    CHECK_WIDGET(lfl)                 ||
	    CHECK_WIDGET(pst)                 ||
	    CHECK_WIDGET(pfl1)                ||
	    CHECK_WIDGET(pfl2)                ||
	    CHECK_WIDGET(dst)                 ||
	    CHECK_WIDGET(dfl1)                ||
	    CHECK_WIDGET(dfl2)                ||
	    CHECK_WIDGET(tsw)                 ||
	    CHECK_WIDGET(aover)               ||
	    CHECK_WIDGET(loc)                 ||
	    CHECK_WIDGET(rco)                 ||
	    CHECK_WIDGET(thip)                ||
	    CHECK_WIDGET(tab))
	{
		return false;
	}

	// Set ranges.
	aea_spinbutton->set_range(1, 2);
	pea_spinbutton->set_range(1, 2);
	lhip_pos_spinbutton->set_range(2.10, 2.20);
	rhip_pos_spinbutton->set_range(2.40, 2.50);
	lst->set_range(0.6, 0.96);
	lfl->set_range(0.6, 0.96);
	pst->set_range(0, 6000);
	pfl1->set_range(0, 6000);
	pfl2->set_range(0, 6000);
	dst->set_range(0, 70);
	dfl1->set_range(0, 70);
	dfl2->set_range(0, 70);
	tsw->set_range(0, 1);
	aover->set_range(0, 1);
	loc->set_range(0, 60);
	rco->set_range(0, 1);
	thip->set_range(-10, 10);
	tab->set_range(0, 1);

	// Set default values.
	control_combobox->set_active(1); // Default to "Manual" mode.
	aea_spinbutton->set_value(1.4);
	pea_spinbutton->set_value(1.8);
	lhip_pos_spinbutton->set_value(2.15);
	rhip_pos_spinbutton->set_value(2.45);
	lst->set_value(0.9);
	lfl->set_value(0.7);
	pst->set_value(2000);
	pfl1->set_value(400);
	pfl2->set_value(200);
	dst->set_value(20);
	dfl1->set_value(10);
	dfl2->set_value(10);
	tsw->set_value(0.8);
	aover->set_value(0.2);
	loc->set_value(5);
	rco->set_value(0);
	thip->set_value(0);
	tab->set_value(0);

	// Connect buttons to functions.
	gc_l_button->signal_pressed().connect(sigc::ptr_fun((void(*)())gc_l_pressed));
	gc_l_button->signal_released().connect(sigc::ptr_fun((void(*)())gc_l_released));
	gc_r_button->signal_pressed().connect(sigc::ptr_fun((void(*)())gc_r_pressed));
	gc_r_button->signal_released().connect(sigc::ptr_fun((void(*)())gc_r_released));

	// Set up subscriber and publisher.
	sub = nh.subscribe("ATCEqPoint_status", 0, controllerCallback);
	pub = nh.advertise<atc_eq_point::controller_input>("ATCEqPoint_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_eq_point::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	// Read in parameters from the file and set them on the GUI
	nh.getParam("/atrias_gui/control", (int&) controllerDataOut.control);
	control_combobox->set_active(controllerDataOut.control);
	nh.getParam("/atrias_gui/aea", controllerDataOut.aea);
	aea_spinbutton->set_value(controllerDataOut.aea);
	nh.getParam("/atrias_gui/pea", controllerDataOut.pea);
	pea_spinbutton->set_value(controllerDataOut.pea);
	nh.getParam("/atrias_gui/lhip_pos", controllerDataOut.lhip_pos);
	lhip_pos_spinbutton->set_value(controllerDataOut.lhip_pos);
	nh.getParam("/atrias_gui/rhip_pos", controllerDataOut.rhip_pos);
	rhip_pos_spinbutton->set_value(controllerDataOut.rhip_pos);
	nh.getParam("/atrias_gui/leg_st", controllerDataOut.leg_st);
	lst->set_value(controllerDataOut.leg_st);
	nh.getParam("/atrias_gui/leg_fl", controllerDataOut.leg_fl);
	lfl->set_value(controllerDataOut.leg_fl);
	nh.getParam("/atrias_gui/p_ls", controllerDataOut.p_ls);
	pst->set_value(controllerDataOut.p_ls);
	nh.getParam("/atrias_gui/p_fl", controllerDataOut.p_fl);
	pfl1->set_value(controllerDataOut.p_fl);
	nh.getParam("/atrias_gui/pfl2", controllerDataOut.pfl2);
	pfl2->set_value(controllerDataOut.pfl2);
	nh.getParam("/atrias_gui/d_ls", controllerDataOut.d_ls);
	dst->set_value(controllerDataOut.d_ls);
	nh.getParam("/atrias_gui/d_fl", controllerDataOut.d_fl);
	dfl1->set_value(controllerDataOut.d_fl);
	nh.getParam("/atrias_gui/dfl2", controllerDataOut.dfl2);
	dfl2->set_value(controllerDataOut.dfl2);
	nh.getParam("/atrias_gui/l_fl", controllerDataOut.l_fl);
	thip->set_value(controllerDataOut.l_fl);
	nh.getParam("/atrias_gui/l_st", controllerDataOut.l_st);
	tab->set_value(controllerDataOut.l_st);
	nh.getParam("/atrias_gui/tsw", controllerDataOut.tsw);
	tsw->set_value(controllerDataOut.tsw);
	nh.getParam("/atrias_gui/loc", controllerDataOut.loc);
	loc->set_value(controllerDataOut.loc);
	nh.getParam("/atrias_gui/d_as", controllerDataOut.d_as);
	aover->set_value(controllerDataOut.d_as);
	nh.getParam("/atrias_gui/rco", controllerDataOut.rco);
	rco->set_value(controllerDataOut.rco);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
	nh.setParam("/atrias_gui/control", controllerDataOut.control);
	nh.setParam("/atrias_gui/aea", controllerDataOut.aea);
	nh.setParam("/atrias_gui/pea", controllerDataOut.pea);
	nh.setParam("/atrias_gui/lhip_pos", controllerDataOut.lhip_pos);
	nh.setParam("/atrias_gui/rhip_pos", controllerDataOut.rhip_pos);
	nh.setParam("/atrias_gui/leg_st", controllerDataOut.leg_st);
	nh.setParam("/atrias_gui/leg_fl", controllerDataOut.leg_fl);
	nh.setParam("/atrias_gui/p_ls", controllerDataOut.p_ls);
	nh.setParam("/atrias_gui/p_fl", controllerDataOut.p_fl);
	nh.setParam("/atrias_gui/pfl2", controllerDataOut.pfl2);
	nh.setParam("/atrias_gui/d_ls", controllerDataOut.d_ls);
	nh.setParam("/atrias_gui/d_fl", controllerDataOut.d_fl);
	nh.setParam("/atrias_gui/dfl2", controllerDataOut.dfl2);
	nh.setParam("/atrias_gui/l_fl", controllerDataOut.l_fl);
	nh.setParam("/atrias_gui/l_st", controllerDataOut.l_st);
	nh.setParam("/atrias_gui/tsw", controllerDataOut.tsw);
	nh.setParam("/atrias_gui/loc", controllerDataOut.loc);
	nh.setParam("/atrias_gui/d_as", controllerDataOut.d_as);
	nh.setParam("/atrias_gui/rco", controllerDataOut.rco);
}

//! \brief Update the GUI.
void guiUpdate() {
	controllerDataOut.control  = control_combobox->get_active_row_number();
	controllerDataOut.aea      = aea_spinbutton->get_value();
	controllerDataOut.pea      = pea_spinbutton->get_value();
	controllerDataOut.lhip_pos = lhip_pos_spinbutton->get_value();
	controllerDataOut.rhip_pos = rhip_pos_spinbutton->get_value();
	controllerDataOut.leg_st   = lst->get_value();
	controllerDataOut.leg_fl   = lfl->get_value();
	controllerDataOut.p_ls     = pst->get_value();
	controllerDataOut.p_fl     = pfl1->get_value();
	controllerDataOut.pfl2     = pfl2->get_value();
	controllerDataOut.d_ls     = dst->get_value();
	controllerDataOut.d_fl     = dfl1->get_value();
	controllerDataOut.dfl2     = dfl2->get_value();
	controllerDataOut.l_fl     = thip->get_value();
	controllerDataOut.l_st     = tab->get_value();
	controllerDataOut.tsw      = tsw->get_value();
	controllerDataOut.loc      = loc->get_value();
	controllerDataOut.d_as     = aover->get_value();
	controllerDataOut.rco      = rco->get_value();

	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

