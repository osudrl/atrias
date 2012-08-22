/*
 * atrias_gui.cpp
 *
 * This file creates and manages the main robot control user interface.
 *
 * Original version by Devin Koepl
 *
 *  Created on: May 7, 2012
 *      Author: Michael Anderson
 */

#include <atrias_gui/atrias_gui.h>
#include <cmath>

//! @brief Initializes the GUI and controls.
//! @param argc An integer that is one more than the number of command line arguments.
//! @param argv An array of character pointers containing the command line arguments.
//! @return Returns zero upon successful completion, non-zero if an error occured.
int main (int argc, char **argv) {
    using namespace atrias;
    using namespace atrias::gui;

    ros::init(argc, argv, "atrias_gui");

    ros::NodeHandle nh;

    //atrias_client = nh.serviceClient<atrias_controllers::atrias_srv>("gui_interface_srv");
    atrias_gui_cm_input = nh.subscribe("gui_input", 0, controllerManagerCallback/*, ros::TransportHints().udp()*/);
    atrias_gui_rt_input = nh.subscribe("gui_robot_state_in", 0, rtOpsCallback);
    atrias_gui_cm_output = nh.advertise<atrias_msgs::gui_output>("gui_output", 0);

    go.command = (uint8_t)UserCommand::STOP;
    go.requestedController = "none";
    //"none" is a special keyword that means keep the torques at 0 until we load an actual controller

    controller_loaded = false;
    robotStateInitialized = false;
    newRobotState = false;

    Gtk::Main gtk(argc, argv);

    // Create the relative path to the Glade file.
    std::string glade_gui_path = std::string(argv[0]);

    glade_gui_path = glade_gui_path.substr(0, glade_gui_path.rfind("/bin"));
    glade_gui_path.append("/media/atrias_gui.glade");

    guiPtr = Gtk::Builder::create();
    try {
        guiPtr->add_from_file(glade_gui_path);
    }
    catch (const Glib::FileError& ex) {
        ROS_ERROR("File Error");
        return 1;
        //          ROS_ERROR("FileError: %d", ex.what());
    }
    catch (const Gtk::BuilderError& ex) {
        ROS_ERROR("Builder Error");
        ROS_ERROR(ex.what().c_str());
        return 1;
        //          ROS_ERROR("BuilderError: %d", ex.what());
    }

    // Grab pointers to GUI objects
    guiPtr->get_widget("atrias_window", controller_window);
    if (!controller_window) {
        ROS_ERROR("No ATRIAS Window");
    }
    guiPtr->get_widget("controller_notebook", controller_notebook);
    guiPtr->get_widget("controller_tree", controller_tree_view);

    guiPtr->get_widget("drawing_area", drawing_area);

    guiPtr->get_widget("log_file_chkbox", log_file_chkbox);
    guiPtr->get_widget("log_frequency_spin", log_frequency_spin);
    guiPtr->get_widget("log_file_chooser", log_file_chooser);

    guiPtr->get_widget("restart_button", restart_button);
    guiPtr->get_widget("enable_button", enable_button);
    guiPtr->get_widget("disable_button", disable_button);
    guiPtr->get_widget("estop_button", estop_button);
    guiPtr->get_widget("save_parameters_button", save_parameters_button);
    guiPtr->get_widget("load_parameters_button", load_parameters_button);

    estopStateColor.set_rgb(0xffff, 0x0000, 0x0000);
    disableStateColor.set_rgb(0x0000, 0xc8c8, 0x0000);
    runStateColor.set_rgb(0x0000, 0x8080, 0xffff);

    /*
     * #region Initialize GUI objects
     *
     */

    changeEstopButtonColor(disableStateColor);

    log_frequency_spin->set_range(100, 10000);
    log_frequency_spin->set_increments(100, 500);
    log_frequency_spin->set_value(100);

    /*
     * #end Initialize GUI region
     *
     */

    /*
     * Set up the controller selection list treeview
     */
    Gtk::TreeModelColumnRecord controllerListRecord;

    controllerListRecord.add(controllerListActiveColumn);
    controllerListRecord.add(controllerListNameColumn);
    controllerListRecord.add(controllerListDescColumn);
    controllerListStore = Gtk::ListStore::create(controllerListRecord);

    detect_controllers();

    controller_tree_view->set_model(controllerListStore);
    controller_tree_view->append_column_editable("Active", controllerListActiveColumn);
    controller_tree_view->append_column("Name", controllerListNameColumn);

    controller_tree_view->get_model()->signal_row_changed().connect(sigc::ptr_fun(controller_checkbox_toggled));

    log_file_chooser->set_action(Gtk::FILE_CHOOSER_ACTION_SAVE);

    drawing_allocation = drawing_area->get_allocation();
    /*
     * Connect buttons to functions.
     */
    log_file_chkbox->signal_toggled().connect(sigc::ptr_fun((void(*)())log_chkbox_toggled));
    restart_button->signal_clicked().connect(sigc::ptr_fun((void(*)())restart_robot));
    enable_button->signal_clicked().connect(sigc::ptr_fun((void(*)())enable_motors));
    disable_button->signal_clicked().connect(sigc::ptr_fun((void(*)())disable_motors));
    estop_button->signal_clicked().connect(sigc::ptr_fun((void(*)())estop_button_clicked));
    save_parameters_button->signal_clicked().connect(sigc::ptr_fun((void(*)())save_parameters));
    load_parameters_button->signal_clicked().connect(sigc::ptr_fun((void(*)())load_parameters));
    controller_notebook->signal_switch_page().connect(sigc::ptr_fun((void(*)(GtkNotebookPage*, guint))switch_controllers));
    sigc::connection conn = Glib::signal_timeout().connect(sigc::ptr_fun(callSpinOnce), 20); // 100 is the timeout in milliseconds

    statusGui = new StatusGui(argv[0]);

    gtk.run(*controller_window);
    ROS_INFO("GUI: Running.");

    return 0;
}

namespace atrias {
namespace gui {

bool callSpinOnce() {
    ros::spinOnce();
    if (controller_loaded)
        controllerUpdate();
    if (newRobotState) {
        statusGui->update(rtCycle);
        newRobotState = false;
    }
    return true;
}

void rtOpsCallback(const rt_ops_cycle &cycle) {
    rtCycle = cycle;
    robotStateInitialized = true;
    draw_leg();
    statusGui->update(rtCycle);
    newRobotState = true;
}

//! \brief Update visualization data.
void controllerManagerCallback(const gui_input &gInput) {
    gi = gInput;

    switch ((ControllerManagerError)gi.errorType) {
        case ControllerManagerError::CONTROLLER_PACKAGE_NOT_FOUND: {
            show_error_dialog("Control machine encountered an error loading the controller:\nController package not found");
            break;
        }
    }

    //Make sure that an e-stop hasn't occurred
    if ((ControllerManagerState)gi.status == ControllerManagerState::CONTROLLER_ESTOPPED) {
        estop_button->set_state(Gtk::StateType::STATE_ACTIVE);
        changeEstopButtonColor(estopStateColor);
    }
    else if ((ControllerManagerState)gi.status == ControllerManagerState::NO_CONTROLLER_LOADED) {
        changeEstopButtonColor(disableStateColor);
    }
    else if ((ControllerManagerState)gi.status == ControllerManagerState::CONTROLLER_RUNNING) {
	    changeEstopButtonColor(runStateColor);
    }
    else if ((ControllerManagerState)gi.status == ControllerManagerState::CONTROLLER_STOPPED) {
        changeEstopButtonColor(disableStateColor);
    }
}

void changeEstopButtonColor(Gdk::Color newColor) {
    estop_button->modify_bg(Gtk::StateType::STATE_ACTIVE, newColor);
    estop_button->modify_bg(Gtk::StateType::STATE_NORMAL, newColor);
    estop_button->modify_bg(Gtk::StateType::STATE_SELECTED, newColor);
    estop_button->modify_bg(Gtk::StateType::STATE_PRELIGHT, newColor);
    estop_button->modify_bg(Gtk::StateType::STATE_INSENSITIVE, newColor);
}

void estop_button_clicked() {
    if (controller_loaded && go.command == (uint8_t)UserCommand::RUN) {
        go.command = (uint8_t)UserCommand::E_STOP;
        atrias_gui_cm_output.publish(go);
    }
}

//! @brief Announce for a log file to be created.
void log_chkbox_toggled() {
    if (log_file_chkbox->get_active()) {
        ROS_INFO("GUI: Sending log enable request.");
        go.enableLogging = true;
    }
    else {
        ROS_INFO("GUI: Sending log disable request.");
        go.enableLogging = false;
    }

    // Publish logging request to controller manager.
    atrias_gui_cm_output.publish(go);
}

//! @brief Save GUI parameters to local (GUI machine) controller directories.
void save_parameters() {
    if (controller_loaded) {
        if (controllerSetParameters) {
            // First, set parameters on server.
            controllerSetParameters();

            // Then, dump to file.
            int rosparamPID = fork();
            if (rosparamPID == 0) {   // Child process
                execlp("rosparam", "rosparam", "dump", metadata[controllerName].guiConfigPath.c_str(), "/atrias_gui", NULL);
                exit(127);   // Exit code 127 if command not found.
            }
            ROS_INFO("GUI: Saved GUI settings in %s", metadata[controllerName].guiConfigPath.c_str());
        }
        else {
            ROS_WARN("GUI: Could not set parameters.");
        }
    }
    else {
        ROS_INFO("GUI: Not saving anything because there is no controller loaded.");
    }
}

//! @brief Load GUI parameters from local (GUI machine) controller directories.
void load_parameters() {
    if (controller_loaded) {
        if (controllerGetParameters) {
            // First, load parameters from file onto server.
            int rosparamPID = fork();
            if (rosparamPID == 0) {   // Child process
            	execlp("rosparam", "rosparam", "load", metadata[controllerName].guiConfigPath.c_str(), "/atrias_gui", NULL);
            	exit(127);   // Exit code 127 if command not found.
            }

	    // Once parameters are loaded, get them and update GUI. I wanted to
	    // use nh.hasParam() here, but nh is out of the scope of this
	    // function.   TODO: This could hang if the exec() call above
	    // fails.  I need another check in the while().
            while (!ros::param::has("/atrias_gui")) {}
            controllerGetParameters();
            ROS_INFO("GUI: Loaded GUI settnigs from %s.", metadata[controllerName].guiConfigPath.c_str());
        }
    }
}

//! @brief restarts the robot.
void restart_robot() {
	go.command = (uint8_t)UserCommand::UNLOAD_CONTROLLER;
	atrias_gui_cm_output.publish(go);
	go.command = (uint8_t)UserCommand::STOP;
	atrias_gui_cm_output.publish(go);
}

//! @brief Enables the motors of the robot.
void enable_motors() {
	if (controller_loaded) {
	    go.command = (uint8_t)UserCommand::RUN;
	    atrias_gui_cm_output.publish(go);
	}
}

//! @brief Disables the motors of the robot.
void disable_motors() {
    go.command = (uint8_t)UserCommand::STOP;
    atrias_gui_cm_output.publish(go);
}

//! @brief Change the active controller.but this is the last night we have and we have to get the rooot
void switch_controllers(GtkNotebookPage* page, guint page_num) {
    if (go.command != (uint8_t)UserCommand::STOP)
        controller_notebook->set_current_page(currentControllerID + 1);
    else if (page_num == CONTROLLER_LOAD_PAGE) {
        //Take down the previous controller
        if (controller_loaded && controllerTakedown && controllerUpdate) {
            controllerTakedown();

            // Delete currently loaded GUI parameters.
            int rosparamPID = fork();
            if (rosparamPID == 0) {   // Child process
            	execlp("rosparam", "rosparam", "delete", "/atrias_gui", NULL);
            	exit(127);   // Exit code 127 if command not found.
            }
        }
        controller_loaded = false;
        go.command = (uint8_t)UserCommand::UNLOAD_CONTROLLER;
        go.requestedController = "none";
        atrias_gui_cm_output.publish(go);
        go.command = (uint8_t)UserCommand::STOP;
    }
    else {
    	//Take down the previous controller
    	if (controller_loaded && controllerTakedown && controllerUpdate) {
    		controllerTakedown();

            // Delete currently loaded GUI parameters.
            int rosparamPID = fork();
            if (rosparamPID == 0) {   // Child process
            	execlp("rosparam", "rosparam", "delete", "/atrias_gui", NULL);
            	exit(127);   // Exit code 127 if command not found.
            }

            go.command = (uint8_t)UserCommand::UNLOAD_CONTROLLER;
            atrias_gui_cm_output.publish(go);
    	}

    	controller_loaded = false;

        controllerName = controllerNames[controllerDetectedIDs[page_num - 1]];

    	//Subtract 1 because page 0 doesn't have a controller library
    	void* handle = controllerHandles[controllerName];

        controllerUpdate = (void(*)())dlsym(handle, "guiUpdate");
    	controllerTakedown = (void(*)())dlsym(handle, "guiTakedown");
        controllerGetParameters = (void(*)()) dlsym(handle, "getParameters");
        controllerSetParameters = (void(*)()) dlsym(handle, "setParameters");

    	if (controllerTakedown && controllerUpdate) {
		controller_loaded = true;
    	}
    	else {
    		show_error_dialog("Controller failed to load:\nCould not load library functions");
    	}

        currentControllerID = page_num - 1;
        go.requestedController = controllerName;
        go.command = (uint8_t)UserCommand::STOP;
        atrias_gui_cm_output.publish(go);
    }
}

//! @brief Loads a new controller plugin and creates its GUI.
bool load_controller(std::string name, uint16_t controllerID) {
    if (!controller_loaded) {
        ControllerMetadata cm = metadata[name];

        void *handle = dlopen(cm.guiLibPath.c_str(), RTLD_LAZY);
        if (!handle) {
        	if (cm.loadSuccessful) {
        		show_error_dialog("Failed to open shared library " + cm.guiLibPath +
        				" for controller " + name + "! Please verify your metadata.txt file.");
        	}
        	else  {
        		show_error_dialog("Could not load metadata for controller " + name +
        				" and an attempt to use default values failed!");
            }
        }
        else {
        	if (!controllerResourcesLoaded[controllerID]) {
				if (!cm.loadSuccessful) {
					show_error_dialog("Could not load metadata for controller " + name + "!\n" +
							"Attempting to use default values.");
				}
				Glib::RefPtr<Gtk::Builder> controllerTab = Gtk::Builder::create();
				try {
					controllerTab->add_from_file(cm.guiDescriptionPath);
				}
				catch (const Glib::FileError& ex) {
					show_error_dialog("Could not load GUI for controller " + name + "!\n" +
							"Please verify your metadata.txt and GUI description files.");
					return false;
				}
				catch (const Gtk::BuilderError& ex) {
					show_error_dialog("Could not load GUI for controller " + name + "!\n" +
							"Please verify your metadata.txt and GUI description files.");
					return false;
				}
				Gtk::Label tabLabel;
				tabLabel.set_label(cm.name);
				tabLabel.set_size_request(-1, 16);
				Gtk::Widget *tabContent;
				controllerTab->get_widget(cm.guiTabWidgetName, tabContent);

				if (tabContent) {
					bool (*controllerInit)(Glib::RefPtr<Gtk::Builder> gui) = (bool(*)(Glib::RefPtr<Gtk::Builder> gui))dlsym(handle, "guiInit");
					bool success = controllerInit(controllerTab);
					if (success) {
						controller_notebook->append_page(*tabContent, tabLabel);
						controller_notebook->set_tab_label_packing(tabLabel, false, true, Gtk::PACK_START);
						controllerHandles.insert(pair<std::string, void*>(name, handle));
						controllerTabs.insert(pair<std::string, Gtk::Widget*>(name, tabContent));
						controllerDetectedIDs.push_back(controllerID);
						controllerResourcesLoaded[controllerID] = true;
					}
					else {
						show_error_dialog("Controller initialization function reported an error!\nController "
								+ name + " has not been loaded.");
						return false;
					}
				}
				else {
					show_error_dialog("Could not load GUI for controller " + name + "!\n" +
							"Please verify your metadata.txt and GUI description files.");
					return false;
				}
        	}
        	else { //The controller was already loaded once before in this instance. This means we don't need to do nearly as much.
        		ControllerMetadata cm = metadata[name];
				Gtk::Label tabLabel;
				tabLabel.set_label(cm.name);
				tabLabel.set_size_request(-1, 16);
				controller_notebook->append_page(*controllerTabs[name], tabLabel);
				controller_notebook->set_tab_label_packing(tabLabel, false, true, Gtk::PACK_START);
				controllerDetectedIDs.push_back(controllerID);
        	}
	        return true;
        }
    }
    return false;
}

//! @brief Loads a new controller plugin.
void unload_controller(std::string name) {
	if (!controller_loaded) {
		for (size_t i = 0; i < controllerDetectedIDs.size(); i++) {
			if (controllerNames[controllerDetectedIDs[i]] == name) {
				controller_notebook->remove_page(i + 1);
				controllerDetectedIDs.erase(controllerDetectedIDs.begin() + i);
				break;
			}
		}
	}
}

void detect_controllers() {
	using ros::package::V_string;
	V_string packages;
	ros::package::getAll(packages);

	for (uint16_t i = 0; i < packages.size(); i++) {
		if (packages[i].find("atc_") == 0) {
			controllerList.push_back(packages[i]);
			ControllerMetadata md = loadControllerMetadata(ros::package::getPath(packages[i]), packages[i]);
			metadata.insert(std::pair<std::string, ControllerMetadata>(packages[i], md));
			Gtk::TreeModel::Row row = *(controllerListStore->append());
			row.set_value(0, false);
			row.set_value(1, md.name + " Controller");
			row.set_value(2, "Description: " + md.description + "\n\nAuthor: " + md.author);
			controllerNames.push_back(packages[i]);
			controllerResourcesLoaded.push_back(false);
		}
	}
}

void controller_checkbox_toggled(const Gtk::TreeModel::Path& path, const Gtk::TreeModel::iterator& iter) {
	Gtk::TreeModel::Row row = *iter;
	if (row[controllerListActiveColumn]) {
		uint16_t controllerNum = atoi(path.to_string().c_str());
		if(!load_controller(controllerNames[controllerNum], controllerNum))
			row[controllerListActiveColumn] = false;
	}
	else {
		uint16_t controllerNum = atoi(path.to_string().c_str());
		unload_controller(controllerNames[controllerNum]);
	}
}

void show_error_dialog(std::string message) {
	Gtk::MessageDialog loadFailedDialog(message, false, Gtk::MESSAGE_ERROR);
	loadFailedDialog.run();
}

//! @brief Draws the four legs of Atrias in the simulation (the carrot).
void draw_leg () {
    float segment_length = 125.;
    float short_segment_length = 100.;
    float motor_radius = 70.;

    float start_x = 125.;
    float start_y = 125.;

    drawing_area->get_window()->clear();

    cc = drawing_area->get_window()->create_cairo_context();


    cc->set_line_width(12.0);

	// Draw the leg
    cc->move_to(start_x, start_y);
    // OSU orange 216, 90, 26
    cc->set_source_rgb(0.8471, 0.3529, 0.1020);
    //cc->set_source_rgb(0.8, 0.0, 0.0);
    // A
    cc->rel_line_to(-short_segment_length * cos(rtCycle.robotState.lLeg.halfA.legAngle), short_segment_length * sin(rtCycle.robotState.lLeg.halfA.legAngle));
    // C
    cc->rel_line_to(-segment_length * cos(rtCycle.robotState.lLeg.halfB.legAngle), segment_length * sin(rtCycle.robotState.lLeg.halfB.legAngle));
    // B
    cc->move_to(start_x, start_y);
    cc->rel_line_to(-segment_length * cos(rtCycle.robotState.lLeg.halfB.legAngle), segment_length * sin(rtCycle.robotState.lLeg.halfB.legAngle));
    // D
    cc->rel_line_to(-segment_length * cos(rtCycle.robotState.lLeg.halfA.legAngle), segment_length * sin(rtCycle.robotState.lLeg.halfA.legAngle));
    cc->stroke();

    // Draw the motors
    cc->set_source_rgb(0.0, 0.8, 0.0);
    // A
    cc->move_to(start_x, start_y);
    cc->rel_line_to(-motor_radius * cos(rtCycle.robotState.lLeg.halfA.motorAngle), motor_radius * sin(rtCycle.robotState.lLeg.halfA.motorAngle));
    cc->move_to(start_x, start_y);
    cc->rel_line_to(motor_radius * cos(rtCycle.robotState.lLeg.halfA.motorAngle), -motor_radius * sin(rtCycle.robotState.lLeg.halfA.motorAngle));
    // B
    cc->move_to(start_x, start_y);
    cc->rel_line_to(-motor_radius * cos(rtCycle.robotState.lLeg.halfB.motorAngle), motor_radius * sin(rtCycle.robotState.lLeg.halfB.motorAngle));
    cc->move_to(start_x, start_y);
    cc->rel_line_to(motor_radius * cos(rtCycle.robotState.lLeg.halfB.motorAngle), -motor_radius * sin(rtCycle.robotState.lLeg.halfB.motorAngle));
    cc->stroke();
}

}
}
