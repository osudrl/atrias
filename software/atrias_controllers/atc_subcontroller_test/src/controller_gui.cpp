/*
 * controller_gui.cpp
 *
 * atc_subcontroller_test controller
 *
 *  Created on: Dec 7, 2012
 *      Author: Ryan Van Why
 */

#include <atc_subcontroller_test/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	gui->get_widget("in1", in1);
	gui->get_widget("in2", in2);
	gui->get_widget("in3", in3);
	gui->get_widget("in4", in4);
	gui->get_widget("in5", in5);
	gui->get_widget("in6", in6);
	gui->get_widget("in7", in7);
	gui->get_widget("in8", in8);
	gui->get_widget("in9", in9);
	gui->get_widget("out1", out1);
	gui->get_widget("out2", out2);
	gui->get_widget("out3", out3);
	gui->get_widget("out4", out4);
	gui->get_widget("out5", out5);
	gui->get_widget("out6", out6);
	gui->get_widget("out7", out7);
	gui->get_widget("out8", out8);
	gui->get_widget("out9", out9);

	if ( in1 &&  in2 &&  in3 &&  in4 &&  in5 &&  in6 &&  in7 &&  in8 &&  in9 &&
	     out1 && out2 && out3 && out4 && out5 && out6 && out7 && out8 && out9) {
		// Set ranges.
		in1->set_text("0.0");
		in2->set_text("0.0");
		in3->set_text("0.0");
		in4->set_text("0.0");
		in5->set_text("0.0");
		in6->set_text("0.0");
		in7->set_text("0.0");
		in8->set_text("0.0");
		in9->set_text("0.0");
		out1->set_text("");
		out2->set_text("");
		out3->set_text("");
		out4->set_text("");
		out5->set_text("");
		out6->set_text("");
		out7->set_text("");
		out8->set_text("");
		out9->set_text("");

		// Set up subscriber and publisher.
		sub = nh.subscribe("atc_subcontroller_test_status", 0, controllerCallback);
		pub = nh.advertise<atc_subcontroller_test::controller_input>("atc_subcontroller_test_input", 0);
		return true;
	}
	return false;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_subcontroller_test::controller_status &status) {
	controllerDataIn = status;
}

void setParameters() {
}

// This reads in the text from a textbox and converts it to a double (catching errors).
double readGtkEntryDbl(Gtk::Entry* entry) {
	try {
		return boost::lexical_cast<double>(entry->get_text());
	} catch (boost::bad_lexical_cast &) {
		return 0.0;
	}
}

//! \brief Update the GUI.
void guiUpdate() {
	controllerDataOut.in1 = readGtkEntryDbl(in1);
	controllerDataOut.in2 = readGtkEntryDbl(in2);
	controllerDataOut.in3 = readGtkEntryDbl(in3);
	controllerDataOut.in4 = readGtkEntryDbl(in4);
	controllerDataOut.in5 = readGtkEntryDbl(in5);
	controllerDataOut.in6 = readGtkEntryDbl(in6);
	controllerDataOut.in7 = readGtkEntryDbl(in7);
	controllerDataOut.in8 = readGtkEntryDbl(in8);
	controllerDataOut.in9 = readGtkEntryDbl(in9);

	out1->set_text(boost::lexical_cast<std::string>(controllerDataIn.out1));
	out2->set_text(boost::lexical_cast<std::string>(controllerDataIn.out2));
	out3->set_text(boost::lexical_cast<std::string>(controllerDataIn.out3));
	out4->set_text(boost::lexical_cast<std::string>(controllerDataIn.out4));
	out5->set_text(boost::lexical_cast<std::string>(controllerDataIn.out5));
	out6->set_text(boost::lexical_cast<std::string>(controllerDataIn.out6));
	out7->set_text(boost::lexical_cast<std::string>(controllerDataIn.out7));
	out8->set_text(boost::lexical_cast<std::string>(controllerDataIn.out8));
	out9->set_text(boost::lexical_cast<std::string>(controllerDataIn.out9));

	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

