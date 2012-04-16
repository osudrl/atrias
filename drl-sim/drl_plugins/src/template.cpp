// Andrew Peekema

#include <drl_plugins/invertedPendulum.h>


// Register the controller using the gazebo controller factory
GZ_REGISTER_DYNAMIC_CONTROLLER("invertedPendulum", InvertedPendulum);

////////////////////////////////////////////////////////////////////////////////
// Constructor
InvertedPendulum::InvertedPendulum(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("InvertedPendulum controller requires an Model as its parent");

  Param::Begin(&this->parameters);
  this->bodyName1P = new ParamT<std::string>("bodyName1","link", 1);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
InvertedPendulum::~InvertedPendulum()
{
  delete this->bodyName1P;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller (Get references to bodies)
void InvertedPendulum::LoadChild(XMLConfigNode *node)
{
  this->bodyName1P->Load(node);
  this->bodyName1 = this->bodyName1P->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName1)) == NULL)
    ROS_FATAL("gazebo controller (invertedPendulum) error: bodyName1: %s does not exist\n",bodyName1.c_str());

  this->body1 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName1));

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR("gazebo controller (invertedPendulum) update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void InvertedPendulum::UpdateChild()
{
	Vector3 force_vec;
	float xPos;

	//Use the command ROS_INFO for debugging
    //ROS_INFO("xPos = %f\n", xPos); 

    xPos = this->body1->GetWorldPose().pos.x;
	if ( xPos <= 0.5 ) {
		force_vec = Vector3(0.1, 0., 0.);
	}
    else {
		force_vec = Vector3(-0.1, 0., 0.);
    }

    this->body1->SetForce(force_vec);
}
