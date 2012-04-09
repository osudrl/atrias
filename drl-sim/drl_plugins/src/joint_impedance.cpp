// Devin Koepl

#include <drl_plugins/joint_impedance.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("joint_impedance", JointImpedance);

////////////////////////////////////////////////////////////////////////////////
// Constructor
JointImpedance::JointImpedance(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("JointImpedance controller requires an Model as its parent");

  Param::Begin(&this->parameters);
  this->bodyName1P  						= new ParamT<std::string>("bodyName1","link", 1);
  this->bodyName2P  						= new ParamT<std::string>("bodyName2","link", 1);
	this->translationalStiffnessP = new ParamT<float>("translationalStiffness", 0., 0);
	this->rotationalStiffnessP		= new ParamT<float>("rotationalStiffness", 0., 0);
	this->translationalDampingP		= new ParamT<float>("translationalDamping", 0., 0);
	this->rotationalDampingP			= new ParamT<float>("rotationalDamping", 0., 0);
	this->translationalStictionP	= new ParamT<float>("translationalStiction", 0., 0);
	this->rotationalStictionP			= new ParamT<float>("rotationalStiction", 0., 0);
	this->restLengthP 						= new ParamT<float>("restLength", 0., 0);
	this->restAngleP							= new ParamT<float>("restAngle", 0., 0);
	this->rotationalFrictionP			= new ParamT<float>("rotationalFriction", 0., 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
JointImpedance::~JointImpedance()
{
  delete this->bodyName1P;
	delete this->bodyName2P;
	delete this->translationalStiffnessP;
	delete this->rotationalStiffnessP;
	delete this->translationalDampingP;
	delete this->rotationalDampingP;
	delete this->translationalStictionP;
	delete this->rotationalStictionP;
	delete this->restLengthP;
	delete this->restAngleP;
	delete this->rotationalFrictionP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void JointImpedance::LoadChild(XMLConfigNode *node)
{
  this->bodyName1P->Load(node);
  this->bodyName1 = this->bodyName1P->GetValue();

  this->bodyName2P->Load(node);
  this->bodyName2 = this->bodyName2P->GetValue();

	// Get stiffness parameters
  this->translationalStiffnessP->Load(node);
  this->translationalStiffness = this->translationalStiffnessP->GetValue();

  this->rotationalStiffnessP->Load(node);
  this->rotationalStiffness = this->rotationalStiffnessP->GetValue();

	// Get damping parameters
  this->translationalDampingP->Load(node);
  this->translationalDamping = this->translationalDampingP->GetValue();

  this->rotationalDampingP->Load(node);
  this->rotationalDamping = this->rotationalDampingP->GetValue();

	// Get stiction parameters
  this->translationalStictionP->Load(node);
  this->translationalStiction = this->translationalStictionP->GetValue();

  this->rotationalStictionP->Load(node);
  this->rotationalStiction = this->rotationalStictionP->GetValue();

	// Get friction parameter
  this->rotationalFrictionP->Load(node);
  this->rotationalFriction = this->rotationalFrictionP->GetValue();

	// Get rest length parameters
  this->restLengthP->Load(node);
  this->restLength = this->restLengthP->GetValue();

  this->restAngleP->Load(node);
  this->restAngle = this->restAngleP->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName1)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: bodyName1: %s does not exist\n",bodyName1.c_str());
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName2)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: bodyName2: %s does not exist\n",bodyName1.c_str());

  this->body1 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName1));
  this->body2 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName2));

  // check update rate against world physics update rate.
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void JointImpedance::UpdateChild()
{
	double angle1, angle2;
	float vel1, vel2;
	Vector3 axis;

  this->lock.lock();

	this->body1->GetWorldPose().rot.GetAsAxis(axis, angle1);
	angle1 *= axis.y;

	this->body2->GetWorldPose().rot.GetAsAxis(axis, angle2);
	angle2 *= axis.y;

	vel1 = this->body1->GetRelativeAngularVel().y;
	vel2 = this->body2->GetRelativeAngularVel().y;

    Vector3 torque(0., (vel2 - vel1) * rotationalDamping, 0.);
	
	if ((vel2 - vel1) > 0)
	{
		torque += Vector3(0., rotationalFriction, 0.);
	}
	else if ((vel2 - vel1) < 0)
	{
		torque += Vector3(0., -rotationalFriction, 0.);
	}

	if (torque != torque)
	{
    	ROS_ERROR("gazebo_ros_force controller torque = (%f, %f, %f)", torque[0], torque[1], torque[2]);
		torque = Vector3(0., 0., 0.);
	}

  this->body1->SetTorque(torque);
	this->body2->SetTorque(torque * -1.);

  this->lock.unlock();
}

