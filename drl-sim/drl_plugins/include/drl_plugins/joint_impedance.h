// Devin Koepl

#ifndef JOINT_IMPEDANCE_HH
#define JOINT_IMPEDANCE_HH

#include <algorithm>
#include <assert.h>

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

#include <ros/ros.h>

#include <geometry_msgs/Wrench.h>

#include <drl_library/drl_math.h>

#define MAX_JNT_TRQ 10000.

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

class JointImpedance : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: JointImpedance(Entity *parent);

  /// \brief Destructor
  public: virtual ~JointImpedance();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief call back when a Wrench message is published
  private: void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& wrenchMsg);

  /// \brief A pointer to the parent entity
  private: Model *myParent;

  /// \brief A pointer to the Body
  private: Body *body1, *body2;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock;

  /// \brief inputs
  private: ParamT<std::string> *bodyName1P; 
  private: std::string bodyName1;

  private: ParamT<std::string> *bodyName2P; 
  private: std::string bodyName2;

	// Stiffness
  private: ParamT<float> *translationalStiffnessP; 
  private: float translationalStiffness;

  private: ParamT<float> *rotationalStiffnessP; 
  private: float rotationalStiffness;

	// Damping
  private: ParamT<float> *translationalDampingP; 
  private: float translationalDamping;

  private: ParamT<float> *rotationalDampingP; 
  private: float rotationalDamping;

	// Stiction
  private: ParamT<float> *translationalStictionP; 
  private: float translationalStiction;

  private: ParamT<float> *rotationalStictionP; 
  private: float rotationalStiction;

	// Friction
  private: ParamT<float> *rotationalFrictionP; 
  private: float rotationalFriction;

	// Zero force length and angle
  private: ParamT<float> *restLengthP; 
  private: float restLength;

  private: ParamT<float> *restAngleP; 
  private: float restAngle;
};

}
#endif //JOINT_IMPEDANCE_HH

