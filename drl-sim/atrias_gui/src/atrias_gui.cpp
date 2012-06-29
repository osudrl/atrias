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

//! \brief Update visualization data.
void visCallback(const controller_status &cStatus) {
    cs = cStatus;
    controller_status_initialized = true;
    draw_leg();
}

//! @brief Initializes the GUI and controls.
//! @param argc An integer that is one more than the number of command line arguments.
//! @param argv An array of character pointers containing the command line arguments.
//! @return Returns zero upon successful completion, non-zero if an error occured.
int main (int argc, char **argv) {
    ros::init(argc, argv, "atrias_gui");
    ros::NodeHandle nh;

    //atrias_client = nh.serviceClient<atrias_controllers::atrias_srv>("gui_interface_srv");
    atrias_gui_sub = nh.subscribe("controller_status_50_hz", 0, visCallback);
    atrias_gui_pub = nh.advertise<atrias_msgs::controller_input>("controller_input", 10);

    ci.command = CMD_DISABLE;
    ci.controller_requested = "none";
    //"none" is a special keyword that means keep the torques at 0 until we load an actual controller

    controller_loaded = false;
    controller_status_initialized = false;
    byteArraysInitialized = false;

    // Subscribe to service named "data_subscriber_srv".
    datalog_client = nh.serviceClient<atrias_msgs::data_log_srv>("data_log_srv");

    Gtk::Main gtk(argc, argv);

    // Create the relative path to the Glade file.
    std::string glade_gui_path = std::string(argv[0]);

    glade_gui_path = glade_gui_path.substr(0, glade_gui_path.rfind("/bin"));
    glade_gui_path.append("/media/atrias_gui.glade");

    gui = Gtk::Builder::create();
    try {
        gui->add_from_file(glade_gui_path);
    }
    catch (const Glib::FileError& ex) {
        ROS_ERROR("File Error");
        return 1;
        //            ROS_ERROR("FileError: %d", ex.what());
    }
    catch (const Gtk::BuilderError& ex) {
        ROS_ERROR("Builder Error");
        ROS_ERROR(ex.what().c_str());
        return 1;
        //            ROS_ERROR("BuilderError: %d", ex.what());
    }

    // Grab pointers to GUI objects
    gui->get_widget("atrias_window", controller_window);
    if (!controller_window) {
        ROS_ERROR("No ATRIAS Window");
    }
    gui->get_widget("controller_notebook", controller_notebook);
    gui->get_widget("controller_tree", controller_tree_view);

    gui->get_widget("drawing_area", drawing_area);

    gui->get_widget("log_file_chkbox", log_file_chkbox);
    gui->get_widget("log_frequency_spin", log_frequency_spin);
    gui->get_widget("log_file_chooser", log_file_chooser);

    gui->get_widget("restart_button", restart_button);
    gui->get_widget("enable_button", enable_button);
    gui->get_widget("disable_button", disable_button);

    /*
     * #region Initialize GUI objects
     *
     */
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
    controller_notebook->signal_switch_page().connect(sigc::ptr_fun((void(*)(GtkNotebookPage*, guint))switch_controllers));

    sigc::connection conn = Glib::signal_timeout().connect(sigc::ptr_fun(poke_controller), 50); // 50 is the timeout in milliseconds

    statusGui = new StatusGui(argv[0]);

    gtk.run(*controller_window);
    ROS_INFO("GUI: Running.");

    return 0;
}

//! @brief Announce for a log file to be created.
void log_chkbox_toggled() {
    if (log_file_chkbox->get_active()) {   // If checkbox is checked...
        ROS_INFO("GUI: Sending log enable request.");
        if (log_file_chooser->get_filename() != "") {
            dls.request.logfilename = log_file_chooser->get_filename();   // Specify name to use for logfile.
        }
        dls.request.isLogging = true;
    }
    else {
        ROS_INFO("GUI: Sending log disable request.");
        log_file_chooser->unselect_all();
        dls.request.isLogging = false;
    }

    datalog_client.call(dls);   // Call the service with request and receive response.

    if (dls.response.logfilename != "") {
        log_file_chooser->set_filename(dls.response.logfilename);   // Set FileChooserButton filename to response.
    }
}

//! @brief restarts the robot.
void restart_robot() {
    ci.command = CMD_RESTART;
    atrias_gui_pub.publish(ci);
}

//! @brief Enables the motors of the robot.
void enable_motors() {
    if (controller_loaded)
        ci.command = CMD_RUN;
}

//! @brief Disables the motors of the robot.
void disable_motors() {
    ci.command = CMD_DISABLE;
    atrias_gui_pub.publish(ci);
}

//! @brief Change the active controller.
void switch_controllers(GtkNotebookPage* page, guint page_num) {
    if (ci.command != CMD_DISABLE)
        controller_notebook->set_current_page(currentControllerID + 1);
    else if (page_num == CONTROLLER_LOAD_PAGE) {
        controller_loaded = false;
        controller_status_initialized = false;
    }
    else {
        //Take down the previous controller
        if (controller_loaded && controllerTakedown)
            controllerTakedown();

        controller_loaded = false;
        controller_status_initialized = false;

        //Free the old ByteArrays
        if (byteArraysInitialized) {
            FREE_BYTE_ARRAY(cInput);
            FREE_BYTE_ARRAY(cStatus);
        }

        std::string controllerName = controllerNames[controllerDetectedIDs[page_num - 1]];

        //Subtract 1 because page 0 doesn't have a controller library
        void* handle = controllerHandles[controllerName];

        controllerUpdate = (void(*)(robot_state, ByteArray, ByteArray&))dlsym(handle, "guiUpdate");
        controllerStandby = (void(*)(robot_state))dlsym(handle, "guiStandby");
        controllerTakedown = (void(*)())dlsym(handle, "guiTakedown");

        ControllerInitResult cir = controllerInitResults[controllerName];
        cInput = NEW_BYTE_ARRAY(cir.controllerInputSize);
        cStatus = NEW_BYTE_ARRAY(cir.controllerStatusSize);
        byteArraysInitialized = true;

        if (controllerUpdate && controllerStandby && controllerTakedown) {
                controller_loaded = true;
        }
        else {
            show_error_dialog("Controller failed to load:\nCould not load library functions");
        }

        currentControllerID = page_num - 1;
        ci.controller_requested = controllerName;
        atrias_gui_pub.publish(ci);
    }
}

//! @brief Loads a new controller plugin and creates its GUI.
bool load_controller(std::string name, uint16_t controllerID) {
    if (!controller_loaded) {
        controller_metadata cm = controllerMetadata[name];

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
                    ControllerInitResult (*controllerInit)(Glib::RefPtr<Gtk::Builder> gui) = (ControllerInitResult(*)(Glib::RefPtr<Gtk::Builder> gui))dlsym(handle, "guiInit");
                    ControllerInitResult cir = controllerInit(controllerTab);
                    if (!cir.error) {
                        controller_notebook->append_page(*tabContent, tabLabel);
                        controller_notebook->set_tab_label_packing(tabLabel, false, true, Gtk::PACK_START);
                        controllerHandles.insert(pair<std::string, void*>(name, handle));
                        controllerTabs.insert(pair<std::string, Gtk::Widget*>(name, tabContent));
                        controllerInitResults.insert(pair<std::string, ControllerInitResult>(name, cir));
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
                controller_metadata cm = controllerMetadata[name];
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
        if (packages[i].find("ac_") == 0) {
            controllerList.push_back(packages[i]);
            controller_metadata md = loadControllerMetadata(ros::package::getPath(packages[i]), packages[i]);
            controllerMetadata.insert(std::pair<std::string, controller_metadata>(packages[i], md));
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

//! @brief Probes and updates the active controller then sends updated commands to the robot.
bool poke_controller() {
    ros::spinOnce();

    if (ci.command == CMD_RESTART)
        ci.command = CMD_DISABLE;

    if(cs.cssm_state == CSSM_STATE_LOAD_FAIL) {
        show_error_dialog("Error: The controller switcher failed to load the controller!");
        ci.command = CMD_DISABLE;
    }

    if (controller_loaded && controller_status_initialized) {
        if (ci.command == CMD_RUN) {
            VECTOR_TO_BYTE_ARRAY(cs.status, cStatus);
            controllerUpdate(cs.state, cStatus, cInput);
            byteArrayToVector(cInput, ci.controller_input);

            atrias_gui_pub.publish(ci);
        }
        else if (ci.command == CMD_DISABLE) {
            controllerStandby(cs.state);
        }
    }

    statusGui->update(cs);

    return true;
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
    cc->rel_line_to(short_segment_length * cos(cs.state.leg_angleA), -short_segment_length * sin(cs.state.leg_angleA));
    // C
    cc->rel_line_to(segment_length * cos(cs.state.leg_angleB), -segment_length * sin(cs.state.leg_angleB));
    // B
    cc->move_to(start_x, start_y);
    cc->rel_line_to(segment_length * cos(cs.state.leg_angleB), -segment_length * sin(cs.state.leg_angleB));
    // D
    cc->rel_line_to(segment_length * cos(cs.state.leg_angleA), -segment_length * sin(cs.state.leg_angleA));
    cc->stroke();

    // Draw the motors
    cc->set_source_rgb(0.0, 0.8, 0.0);
    // A
    cc->move_to(start_x, start_y);
    cc->rel_line_to(motor_radius * cos(cs.state.motor_angleA), -motor_radius * sin(cs.state.motor_angleA));
    cc->move_to(start_x, start_y);
    cc->rel_line_to(-motor_radius * cos(cs.state.motor_angleA), motor_radius * sin(cs.state.motor_angleA));
    // B
    cc->move_to(start_x, start_y);
    cc->rel_line_to(motor_radius * cos(cs.state.motor_angleB), -motor_radius * sin(cs.state.motor_angleB));
    cc->move_to(start_x, start_y);
    cc->rel_line_to(-motor_radius * cos(cs.state.motor_angleB), motor_radius * sin(cs.state.motor_angleB));
    cc->stroke();
}

