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

// For error messages
#include <ros/ros.h>

using namespace gazebo;

class InvertedPendulum : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: InvertedPendulum(Entity *parent);

  /// \brief Destructor
  public: virtual ~InvertedPendulum();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief A pointer to the parent entity
  private: Model *myParent;

  /// \brief A pointer to the Body
  private: Body *body1;

  /// \brief Inputs
  private: ParamT<std::string> *bodyName1P; 
  private: std::string bodyName1;

};

