// Andrew Peekema

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Body.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Param.hh>
#include <gazebo/Model.hh>
#include <gazebo/PhysicsFactory.hh>
#include <gazebo/Pose3d.hh>

// For error messages
#include <ros/ros.h>

// For Math
#include <math.h>

// File Operations
#include <iostream>
#include <fstream>

using namespace gazebo;
using namespace std;

class InvertedPendulumWithToe : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: InvertedPendulumWithToe(Entity *parent);

  /// \brief Destructor
  public: virtual ~InvertedPendulumWithToe();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief A pointer to the parent entity
  private: Model *myParent;

  /// \brief A pointer to the Body
  private: Body *body1, *body2;

  /// \brief A mutex to make sure states don't change inbetween ROS message callbacks
  boost::mutex lock;

  /// \brief Inputs
  private: ParamT<std::string> *bodyName1P; 
  private: std::string bodyName1;

  private: ParamT<std::string> *bodyName2P; 
  private: std::string bodyName2;

  /// \brief Controller variables
  private: Pose3d orig_pose, pose;
  private: Vector3 force_on_mass, force_on_toe;
  private: float zPosM, xPosM, zVelM, zVelMPrev;
  private: float zPosF, xPosF, zVelF;
  private: float pi, l0, alpha, k;
  private: float F, phi, xfp, l, stance, touchDown;
  private: float Fx, Fz;

  template <typename T> int sgn(T val) {
      return (T(0) < val) - (val < T(0));
      }
  
  private: ofstream file;

};

