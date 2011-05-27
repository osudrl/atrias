// Devin Koepl

#include <atrias_controllers/ecat_interface.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ecat_interface", EcatInterface);	

////////////////////////////////////////////////////////////////////////////////
// Constructor
EcatInterface::EcatInterface(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("ControllerWrapper controller requires an Model as its parent");

  Param::Begin(&this->parameters);
  this->bodyNameP	= new ParamT<std::string>("bodyName", "link", 1);
  this->motorANameP	= new ParamT<std::string>("motorAName", "link", 1);
	this->motorBNameP	= new ParamT<std::string>("motorBName", "link", 1);
  this->legANameP	= new ParamT<std::string>("legAName", "link", 1);
	this->legBNameP	= new ParamT<std::string>("legBName", "link", 1);
  Param::End();

	/****************************************************************/
	// Setup shared memory access.

	// Locate the segment.
	if ((this->shmid = shmget(GAZEBO_SHM_KEY, SHMSZ, 0666)) < 0) {
	    ROS_ERROR("shmget");
	}

	// Now we attach the segment to our data space.
	if ((this->gazebo_shm = (SimulationEcatInterface *)shmat(this->shmid, NULL, 0)) == (SimulationEcatInterface *) -1) {
	    ROS_ERROR("shmat");
	}

	last_time = 0.;
	last_torqueA = 0;
	last_torqueB = 0;

	/****************************************************************/
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
EcatInterface::~EcatInterface()
{
	delete this->bodyNameP;
  delete this->motorANameP;
	delete this->motorBNameP;
	delete this->legANameP;
	delete this->legBNameP;

	delete this->controller_input;
	delete this->controller_output;
	delete this->controller_state;

	delete this->gazebo_shm;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void EcatInterface::LoadChild(XMLConfigNode *node)
{
//	Simulator::Instance()->SetPaused(true);

	this->bodyNameP->Load(node);
	this->bodyName = this->bodyNameP->GetValue();

	this->motorANameP->Load(node);
	this->motorAName = this->motorANameP->GetValue();

	this->motorBNameP->Load(node);
	this->motorBName = this->motorBNameP->GetValue();

	this->legANameP->Load(node);
	this->legAName = this->legANameP->GetValue();

	this->legBNameP->Load(node);
	this->legBName = this->legBNameP->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: bodyName: %s does not exist\n", bodyName.c_str());
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->motorAName)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: motorAName: %s does not exist\n", motorAName.c_str());
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->motorBName)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: motorBName: %s does not exist\n", motorBName.c_str());
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->legAName)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: legAName: %s does not exist\n", legAName.c_str());
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->legBName)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: legBName: %s does not exist\n", legBName.c_str());

  this->body = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));
  this->motorA = dynamic_cast<Body*>(this->myParent->GetBody(motorAName));
	this->motorB = dynamic_cast<Body*>(this->myParent->GetBody(motorBName));
  this->legA = dynamic_cast<Body*>(this->myParent->GetBody(legAName));
  this->legB = dynamic_cast<Body*>(this->myParent->GetBody(legBName));

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void EcatInterface::UpdateChild()
{
	double angle;
	Vector3 axis;

	this->lock.lock();

	if (this->gazebo_shm->lock)
	{
		this->lock.unlock();
		return;
		// ROS_WARN("Waiting for SHM.");
	}
	this->gazebo_shm->lock = true;

	// Reset the timestep counter when the motor torques change.
	if ((last_torqueA != gazebo_shm->atrias_input.ucontroller_inputA.motor_torque) || (last_torqueB != gazebo_shm->atrias_input.ucontroller_inputB.motor_torque))
	{
		this->last_time = Simulator::Instance()->GetSimTime().Double();
	}

	gazebo_shm->atrias_output.ucontroller_outputA.timestep = DISCRETIZE_16((Simulator::Instance()->GetSimTime().Double() - this->last_time),
		MIN_TIMESTEP, MAX_TIMESTEP);

	//ROS_INFO("mvA = %.3f,\tmvB = %.3f", this->motorA->GetWorldAngularVel().y, this->motorB->GetWorldAngularVel().y);

	// Update encoder angles.
	this->motorA->GetWorldPose().rot.GetAsAxis(axis, angle);
	gazebo_shm->atrias_output.ucontroller_outputA.transmission_position = DISCRETIZE_32(angle * axis.y + PI/4., 
		MIN_ROTATION_ANGLE, MAX_ROTATION_ANGLE);

	this->motorB->GetWorldPose().rot.GetAsAxis(axis, angle);
	gazebo_shm->atrias_output.ucontroller_outputB.transmission_position = DISCRETIZE_32(angle * axis.y + 3.*PI/4., 
		MIN_ROTATION_ANGLE, MAX_ROTATION_ANGLE);

	this->legA->GetWorldPose().rot.GetAsAxis(axis, angle);
	gazebo_shm->atrias_output.ucontroller_outputA.spring_deflection = DISCRETIZE_16(angle * axis.y + PI/4.,
		MIN_ROTATION_ANGLE, MAX_ROTATION_ANGLE);

	this->legB->GetWorldPose().rot.GetAsAxis(axis, angle);
	gazebo_shm->atrias_output.ucontroller_outputB.spring_deflection = DISCRETIZE_16(angle * axis.y + 3.*PI/4.,
		MIN_ROTATION_ANGLE, MAX_ROTATION_ANGLE);

	last_torqueA = gazebo_shm->atrias_input.ucontroller_inputA.motor_torque;
	last_torqueB = gazebo_shm->atrias_input.ucontroller_inputB.motor_torque;

	this->gazebo_shm->fresh = true;

	this->gazebo_shm->lock = false;

	this->motorA->SetTorque(Vector3(0., UNDISCRETIZE_16(gazebo_shm->atrias_input.ucontroller_inputA.motor_torque, 
		MIN_MOTOR_TORQUE, MAX_MOTOR_TORQUE) * GEAR_RATIO, 0.));
	this->motorB->SetTorque(Vector3(0., UNDISCRETIZE_16(gazebo_shm->atrias_input.ucontroller_inputB.motor_torque, 
		MIN_MOTOR_TORQUE, MAX_MOTOR_TORQUE) * GEAR_RATIO, 0.));

  this->lock.unlock();
}
