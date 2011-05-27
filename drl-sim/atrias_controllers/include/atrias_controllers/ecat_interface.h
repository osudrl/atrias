// Devin Koepl

#ifndef ECAT_INTERFACE_HH
#define ECAT_INTERFACE_HH

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
//#include <algorithm>
#include <assert.h>
#include <limits.h>

#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Body.hh>
#include <gazebo/Pose3d.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Param.hh>
#include <gazebo/Model.hh>
#include <gazebo/Quatern.hh>
#include <gazebo/Mass.hh>

#include <atrias/controller.h>
#include <atrias/atrias_parameters.1.h>
#include <atrias/simulation_ecat_interface.h>

#include <gui/controller_types.h>
#include <gui/atrias_srv.h>

#include <drl_library/discretize.h>
#include <drl_library/drl_math.h>

#define SHMSZ     			27
#define GAZEBO_SHM_KEY 	5678

#define MIN_ENCODER_VALUE 0
#define MAX_ENCODER_VALUE 8192

namespace gazebo
{
/** \defgroup GazeboRosForce Plugin XML Reference and Example

  Example Usage:
  \verbatim
  <model:physical name="box_model">
    <body:empty name="box_body">
     ...
    </body:empty>
    <controller:gazebo_ros_force name="box_force_controller" plugin="libgazebo_ros_force.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>15.0</updateRate>
        <bodyName>box_body</bodyName>
    </controller:gazebo_ros_force>
  </model:phyiscal>
  \endverbatim
 
\{
*/

class EcatInterface : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: EcatInterface(Entity *parent);

  /// \brief Destructor
  public: virtual ~EcatInterface();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief A pointer to the parent entity
  private: Model *myParent;

  /// \brief A pointer to the Body
	private: Body *body;
  private: Body *motorA, *motorB;
	private: Body *legA, *legB;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock;

  /// \brief inputs
  private: ParamT<std::string> *bodyNameP; 
  private: std::string bodyName;

  private: ParamT<std::string> *motorANameP; 
  private: std::string motorAName;

  private: ParamT<std::string> *motorBNameP; 
  private: std::string motorBName;

  private: ParamT<std::string> *legANameP; 
  private: std::string legAName;

  private: ParamT<std::string> *legBNameP; 
  private: std::string legBName;

	private: int shmid;
	private: SimulationEcatInterface *gazebo_shm;
	private: void discretize_io();

	private: ControllerInput *controller_input;
	private: ControllerOutput *controller_output;
	private: ControllerState *controller_state;

	private: float last_time;
	private: int last_torqueA;
	private: int last_torqueB;
};

}
#endif

