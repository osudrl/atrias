// Devin Koepl

#ifndef ATRIAS_GUI_H_
#define ATRIAS_GUI_H_

#include <stdlib.h>
#include <stdarg.h>
#include <gtkmm.h>
#include <cairomm/context.h>
#include <glib/gtypes.h>
#include <sigc++/connection.h>

#include <time.h>
#include <string.h>
#include <vector>
#include <map>
#include <dlfcn.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <atrias_control/ucontroller.h>
#include <atrias_control/controller.h>
#include <atrias_control/controller_metadata.h>

#include <atrias_msgs/controller_input.h>
#include <atrias_msgs/controller_status.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/data_log_srv.h>

#include <drl_library/drl_math.h>
#include <atrias_gui/StatusGui.h>

#define CONTROLLER_LOAD_PAGE 0

using namespace atrias_msgs;

// GUI objects
Glib::RefPtr<Gtk::Builder> gui;
Gtk::Window *controller_window;

Gtk::Notebook *controller_notebook;
Gtk::TreeView *controller_tree_view;

Gtk::DrawingArea *drawing_area;

Gtk::CheckButton *log_file_chkbox;
Gtk::SpinButton *log_frequency_spin;
Gtk::FileChooserButton *log_file_chooser;

Gtk::Button *restart_button,
            *enable_button,
            *disable_button;

Cairo::RefPtr<Cairo::Context> cc;
Gtk::Allocation drawing_allocation;

ros::ServiceClient datalog_client;

ros::Subscriber atrias_gui_sub;
ros::Publisher atrias_gui_pub;

StatusGui *statusGui;

controller_status cs;
controller_input ci;

data_log_srv dls;

Glib::RefPtr<Gtk::ListStore> controllerListStore;
Gtk::TreeModelColumn<bool> controllerListActiveColumn;
Gtk::TreeModelColumn<std::string> controllerListNameColumn;
Gtk::TreeModelColumn<std::string> controllerListDescColumn;

long nextLogTime;

std::vector<std::string> controllerList; //List of all available controllers

std::vector<std::string> controllerNames; //This maps controllers to controller package names in the order they were detected
std::vector<uint16_t> controllerDetectedIDs; //This maps controller page numbers to their detection IDs
/*
 * Maps controller detection ID's to whether a controller has already had its resources loaded.
 * This is necessary to allow resources to be preserved between loads while preventing memory leaks
 */
std::vector<bool> controllerResourcesLoaded;
std::map<std::string, ControllerInitResult> controllerInitResults;
std::map<std::string, void*> controllerHandles; //The pointers needed to access the controller GUI libraries
std::map<std::string, controller_metadata> controllerMetadata;
std::map<std::string, Gtk::Widget*> controllerTabs;
uint8_t currentControllerID;

ControllerInitResult (*controllerInit)(Glib::RefPtr<Gtk::Builder> gui);
void (*controllerUpdate)(robot_state, ByteArray, ByteArray&);
void (*controllerStandby)(robot_state);
void (*controllerTakedown)();

bool controller_loaded;
bool controller_status_initialized;

bool byteArraysInitialized;
ByteArray cInput; //Stores controller inputs from GUI plugins
ByteArray cStatus; //Stores controller status from GUI plugins

bool poke_controller();
bool load_controller(std::string name, uint16_t controllerID);
void unload_controller(std::string name);
void detect_controllers();
void controller_checkbox_toggled(const Gtk::TreeModel::Path& path, const Gtk::TreeModel::iterator& iter);

void show_error_dialog(std::string message);

void log_chkbox_toggled();

void restart_robot();
void enable_motors();
void disable_motors();

void switch_controllers(GtkNotebookPage *, guint);

void draw_leg();

#endif
