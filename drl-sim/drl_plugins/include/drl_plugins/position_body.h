// Devin Koepl

#ifndef PositionBody_HH
#define PositionBody_HH

#include <algorithm>
#include <assert.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

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
#include <gazebo/Vector3.hh>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <drl_plugins/position_body_srv.h>

// PD controller gains.
#define KP 10000.
#define KD 1000.

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

  class PositionBody : public Controller
  {
  public:
    //! @brief Constructor
    //! @param parent The parent entity, must be a Model or a Sensor
    PositionBody(Entity *parent);

    //! @brief Destructor
    virtual ~PositionBody();

  protected:
    //! @brief Load the controller
    //! @param node XML config node
    virtual void LoadChild(XMLConfigNode *node);

    //! @brief Update the controller
    virtual void UpdateChild();

    //! @brief Initialize the controller
    virtual void InitChild();

    //! @brief Destroy the controller
    virtual void FiniChild();

  private:
    //! @brief A pointer to the parent entity
    Model *myParent;

    //! @brief A pointer to the Body
    Body *body;

    //! @brief A mutex to lock access to fields that are used in ROS message callbacks
    boost::mutex lock;

    //! @brief inputs
    ParamT<std::string> *bodyNameP;
    std::string bodyName;

    bool hold_robot;
    Pose3d desired_pose;

    bool xIsConstrained;
    bool yIsConstrained;
    bool zIsConstrained;

    // GUI Interface
    bool position_body_gui_callback(drl_plugins::position_body_srv::Request&, drl_plugins::position_body_srv::Response&);
    ros::NodeHandle *nh;
    ros::ServiceServer position_body_srv;

    // Custom Callback Queue
    ros::CallbackQueue queue;
    boost::thread* callback_queuethread;
    void QueueThread();
  };

}
#endif
