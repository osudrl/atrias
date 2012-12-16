// Devin Koepl

#ifndef ATRIAS_GUI_H_
#define ATRIAS_GUI_H_

#include <stdlib.h>
#include <stdarg.h>
#include <gtkmm.h>
#include <cairomm/context.h>
#include <glib.h>
#include <sigc++/connection.h>

#include <time.h>
#include <string.h>
#include <vector>
#include <map>
#include <dlfcn.h>
#include <unistd.h>   // For exec().

#include <ros/ros.h>
#include <ros/package.h>

#include <atrias_shared/globals.h>
#include <atrias_shared/controller_metadata.h>
#include <atrias_shared/drl_math.h>

#include <atrias_msgs/rt_ops_cycle.h>
#include <atrias_msgs/gui_input.h>
#include <atrias_msgs/gui_output.h>
#include <atrias_msgs/log_request.h>

#include <atrias_gui/StatusGui.h>

#define CONTROLLER_LOAD_PAGE 0

using namespace atrias_msgs;

namespace atrias {
using namespace controllerManager;
using namespace controllerMetadata;
namespace gui {

Gdk::Color estopStateColor;
Gdk::Color disableStateColor;
Gdk::Color runStateColor;

// GUI objects
Glib::RefPtr<Gtk::Builder> guiPtr;
Gtk::Window *controller_window;

Gtk::Notebook *controller_notebook;
Gtk::TreeView *controller_tree_view;

Gtk::DrawingArea *drawing_area;

Gtk::CheckButton *log_file_chkbox;
Gtk::SpinButton *log_frequency_spin;
Gtk::FileChooserButton *log_file_chooser;

Gtk::Button *restart_button,
            *enable_button,
            *disable_button,
            *estop_button,
            *save_parameters_button,
            *load_parameters_button;

Gtk::EventBox *estop_eventbox;

Cairo::RefPtr<Cairo::Context> cc;
Gtk::Allocation drawing_allocation;

ros::Subscriber atrias_gui_rt_input;
ros::Subscriber atrias_gui_cm_input;
ros::Publisher atrias_gui_cm_output;
ros::Publisher atrias_gui_logger_output;

gui_input gi;
gui_output go;
rt_ops_cycle rtCycle;
log_request logRequest;

StatusGui *statusGui;

Glib::RefPtr<Gtk::ListStore> controllerListStore;
Gtk::TreeModelColumn<bool> controllerListActiveColumn;
Gtk::TreeModelColumn<std::string> controllerListNameColumn;
Gtk::TreeModelColumn<std::string> controllerListDescColumn;

std::vector<std::string> controllerList; //List of all available controllers

std::vector<std::string> controllerNames; //This maps controllers to controller package names in the order they were detected
std::vector<uint16_t> controllerDetectedIDs; //This maps controller page numbers to their detection IDs
/*
 * Maps controller detection ID's to whether a controller has already had its resources loaded.
 * This is necessary to allow resources to be preserved between loads while preventing memory leaks
 */
std::vector<bool> controllerResourcesLoaded;
std::map<std::string, void*> controllerHandles; //The pointers needed to access the controller GUI libraries
std::map<std::string, ControllerMetadata> metadata;
std::string controllerName;   // Name of currently loaded controller. This is declared here so GUI parameter load/delete will work in switch_controllers().
std::map<std::string, Gtk::Widget*> controllerTabs;
uint8_t currentControllerID;

bool (*controllerInit)(Glib::RefPtr<Gtk::Builder> guiPtr);
void (*controllerUpdate)();
void (*controllerTakedown)();
void (*controllerGetParameters)();
void (*controllerSetParameters)();

bool controller_loaded;
bool robotStateInitialized;
bool newRobotState;

bool load_controller(std::string name, uint16_t controllerID);
void unload_controller(std::string name);
void detect_controllers();
void rtOpsCallback(const rt_ops_cycle &cycle);
void controllerManagerCallback(const gui_input &gInput);
void controller_checkbox_toggled(const Gtk::TreeModel::Path& path, const Gtk::TreeModel::iterator& iter);

void show_error_dialog(std::string message);
bool callSpinOnce();
void changeEstopButtonColor(Gdk::Color newColor);
void estop_button_clicked();
void log_chkbox_toggled();
void save_parameters();
void load_parameters();

void restart_robot();
void enable_motors();
void disable_motors();

void takedown_current_controller();

void switch_controllers(GtkNotebookPage *, guint);

void draw_leg();

} // namespace gui
} // namespace atrias
#endif

