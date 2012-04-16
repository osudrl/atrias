// Devin Koepl

#include <drl_plugins/invertedPendulum.h>

using namespace gazebo;

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
  this->bodyName1P  						= new ParamT<std::string>("bodyName1","link", 1);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
InvertedPendulum::~InvertedPendulum()
{
  delete this->bodyName1P;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void InvertedPendulum::LoadChild(XMLConfigNode *node)
{
  this->bodyName1P->Load(node);
  this->bodyName1 = this->bodyName1P->GetValue();


  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName1)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: bodyName1: %s does not exist\n",bodyName1.c_str());

  this->body1 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName1));

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void InvertedPendulum::UpdateChild()
{
	//double angle1, angle2;
	Vector3 force_vec;
	float force, xPos;

  //this->lock.lock();

	//this->body1->GetWorldPose().rot.GetAsAxis(axis, angle1);

//	ROS_INFO("B1 = %.3f, B2 = %.3f\n", body1_pose.pos.z, body2_pose.pos.z); 
//	ROS_INFO("%s = %.3f, %s = %.3f\n", this->bodyName1.c_str(), body1_rot.y, this->bodyName2.c_str(), body2_rot.y); 

    xPos = this->body1->GetWorldPose().pos.x;
	if ( xPos <= 0.5 ) {
		force_vec = Vector3(0.1, 0., 0.);
	}
    else {
		force_vec = Vector3(-0.1, 0., 0.);
    }

//	Vector3 torque_vec(0., (angle2 - angle1) * rotationalStiffness, 0.);

//  Vector3 force(this->wrench.force.x,this->wrench.force.y,this->wrench.force.z);
//  Vector3 torque(this->wrench.torque.x,this->wrench.torque.y,this->wrench.torque.z);
//  this->myBody->SetForce(force);

  this->body1->SetForce(force_vec);

  //this->lock.unlock();
}

