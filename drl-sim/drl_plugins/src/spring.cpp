// Devin Koepl

#include <drl_plugins/spring.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("spring", Spring);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Spring::Spring(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Spring controller requires an Model as its parent");

  Param::Begin(&this->parameters);
  this->bodyName1P  						= new ParamT<std::string>("bodyName1","link", 1);
  this->bodyName2P  						= new ParamT<std::string>("bodyName2","link", 1);
	this->translationalStiffnessP = new ParamT<float>("translationalStiffness", 0., 0);
	this->rotationalStiffnessP		= new ParamT<float>("rotationalStiffness", 0., 0);
	this->restLengthP 						= new ParamT<float>("restLength", 0., 0);
	this->restAngleP							= new ParamT<float>("restAngle", 0., 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Spring::~Spring()
{
  delete this->bodyName1P;
	delete this->bodyName2P;
	delete this->translationalStiffnessP;
	delete this->rotationalStiffnessP;
	delete this->restLengthP;
	delete this->restAngleP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Spring::LoadChild(XMLConfigNode *node)
{
  this->bodyName1P->Load(node);
  this->bodyName1 = this->bodyName1P->GetValue();

  this->bodyName2P->Load(node);
  this->bodyName2 = this->bodyName2P->GetValue();

  this->translationalStiffnessP->Load(node);
  this->translationalStiffness = this->translationalStiffnessP->GetValue();

  this->rotationalStiffnessP->Load(node);
  this->rotationalStiffness = this->rotationalStiffnessP->GetValue();

  this->restLengthP->Load(node);
  this->restLength = this->restLengthP->GetValue();

  this->restAngleP->Load(node);
  this->restAngle = this->restAngleP->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName1)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: bodyName1: %s does not exist\n",bodyName1.c_str());
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName2)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: bodyName2: %s does not exist\n",bodyName2.c_str());

  this->body1 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName1));
  this->body2 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName2));

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Spring::UpdateChild()
{
	double angle1, angle2;
	Vector3 axis;
	float torque;

  this->lock.lock();

	this->body1->GetWorldPose().rot.GetAsAxis(axis, angle1);
	angle1 *= axis.y;

	this->body2->GetWorldPose().rot.GetAsAxis(axis, angle2);
	angle2 *= axis.y;

//	ROS_INFO("B1 = %.3f, B2 = %.3f\n", body1_pose.pos.z, body2_pose.pos.z); 
//	ROS_INFO("%s = %.3f, %s = %.3f\n", this->bodyName1.c_str(), body1_rot.y, this->bodyName2.c_str(), body2_rot.y); 

	torque = (angle2 - angle1) * rotationalStiffness;
	if (torque != torque)
	{
		torque = 0.;
	}

	Vector3 torque_vec(0., (angle2 - angle1) * rotationalStiffness, 0.);

//  Vector3 force(this->wrench.force.x,this->wrench.force.y,this->wrench.force.z);
//  Vector3 torque(this->wrench.torque.x,this->wrench.torque.y,this->wrench.torque.z);
//  this->myBody->SetForce(force);

  this->body1->SetTorque(torque_vec);
	this->body2->SetTorque(torque_vec * -1.);

  this->lock.unlock();
}

