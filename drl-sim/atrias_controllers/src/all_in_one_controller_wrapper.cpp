// Devin Koepl

#include <atrias_controllers/all_in_one_controller_wrapper.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("all_in_one_controller_wrapper", AllInOneControllerWrapper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
AllInOneControllerWrapper::AllInOneControllerWrapper(Entity *parent)
    : Controller(parent)
{
//	PhysicsEngine::InitForThread();

  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("ControllerWrapper requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->bodyNameP	= new ParamT<std::string>("bodyName", "link", 1);
  this->motorANameP	= new ParamT<std::string>("motorAName", "link", 1);
	this->motorBNameP	= new ParamT<std::string>("motorBName", "link", 1);
  this->legANameP	= new ParamT<std::string>("legAName", "link", 1);
	this->legBNameP	= new ParamT<std::string>("legBName", "link", 1);
  Param::End();

	this->controller_input				= new ControllerInput();
	this->controller_output				= new ControllerOutput();
	this->controller_state				= new ControllerState();
	this->controller_data					= new ControllerData();

	this->controller_state->state = STATE_INIT;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
AllInOneControllerWrapper::~AllInOneControllerWrapper()
{
	controller_state->state = STATE_FINI;
	//control_switcher_state_machine(controller_input, controller_output, controller_state, controller_data);

	delete this->bodyNameP;
  delete this->motorANameP;
	delete this->motorBNameP;
	delete this->legANameP;
	delete this->legBNameP;

	delete this->controller_input;
	delete this->controller_output;
	delete this->controller_state;
	delete this->controller_data;

	delete this->nh;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void AllInOneControllerWrapper::LoadChild(XMLConfigNode *node)
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

	this->generate_controller_input();
	control_switcher_state_machine(controller_input, controller_output, controller_state, controller_data);
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "gui_interface", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	this->nh = new ros::NodeHandle(); 
  ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<atrias_controllers::atrias_srv>(
      "gui_interface_srv", boost::bind(&AllInOneControllerWrapper::atrias_gui_callback, this, _1, _2), ros::VoidPtr(), &this->queue);
  this->gui_srv = this->nh->advertiseService(aso);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void AllInOneControllerWrapper::UpdateChild()
{
  this->lock.lock();

	this->generate_controller_input();

	// Keep track of last controller output, just in case the new one
	//ControllerOutput = 

	control_switcher_state_machine(this->controller_input, this->controller_output, this->controller_state, this->controller_data);

	/*if ( this->controller_output->motor_torqueA != this->controller_output->motor_torqueA )
	{
		this->controller_output->motor_torqueA = 0.;

		//ROS_WARN("maA = %.3f, mvA = %.3f", controller_input->motor_angleA, controller_input->motor_velocityA);
	}

	if ( this->controller_output->motor_torqueB != this->controller_output->motor_torqueB )
	{
		this->controller_output->motor_torqueB = 0.;
		//ROS_WARN("maB = %.3f, mvB = %.3f", controller_input->motor_angleB, controller_input->motor_velocityB);
	}*/

	//this->controller_output->motor_torqueA = CLAMP( this->controller_output->motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ );
	//this->controller_output->motor_torqueB = CLAMP( this->controller_output->motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ );

	this->motorA->SetTorque(Vector3(0., -this->controller_output->motor_torqueA * GEAR_RATIO, 0.));
	this->motorB->SetTorque(Vector3(0., -this->controller_output->motor_torqueB * GEAR_RATIO, 0.));

  this->lock.unlock();
}

void AllInOneControllerWrapper::generate_controller_input()
{
	double angle;
	Vector3 axis;

	float vel;

	this->body->GetWorldPose().rot.GetAsAxis(axis, angle);
	this->controller_input->body_angle	 = angle * axis.y;
	this->motorA->GetWorldPose().rot.GetAsAxis(axis, angle);
	this->controller_input->motor_angleA = -angle * axis.y + 3.*PI/4. - PI;
	//ROS_INFO("maA = %.3f", angle);
	this->motorB->GetWorldPose().rot.GetAsAxis(axis, angle);
	this->controller_input->motor_angleB = -angle * axis.y + PI/4. + PI;
	//ROS_INFO("maB = %.3f", angle);
	this->legA->GetWorldPose().rot.GetAsAxis(axis, angle);
	this->controller_input->leg_angleA	 = -angle * axis.y + 3.*PI/4. - PI;
	//ROS_INFO("laA = %.3f", angle);
	this->legB->GetWorldPose().rot.GetAsAxis(axis, angle);
	this->controller_input->leg_angleB	 = -angle * axis.y + PI/4. + PI;
	//ROS_INFO("laB = %.3f\n\n", angle);

	this->controller_input->body_ang_vel = this->body->GetWorldAngularVel().y;
	this->controller_input->motor_velocityA = -this->motorA->GetWorldAngularVel().y;
	this->controller_input->motor_velocityB = -this->motorB->GetWorldAngularVel().y;
	this->controller_input->leg_velocityA = -this->legA->GetWorldAngularVel().y;
	this->controller_input->leg_velocityB = -this->legB->GetWorldAngularVel().y;

	this->controller_input->height = this->body->GetWorldPose().pos.z;

	this->controller_input->horizontal_velocity = this->body->GetWorldLinearVel().x;
	this->controller_input->vertical_velocity = this->body->GetWorldLinearVel().z;
}

// Initialize the controller
void AllInOneControllerWrapper::InitChild()
{
  callback_queuethread = new boost::thread(boost::bind(&AllInOneControllerWrapper::QueueThread, this));
}

// Shutdown
void AllInOneControllerWrapper::FiniChild()
{
  // Callback Queue
  queue.clear();
  queue.disable();
  nh->shutdown();
  callback_queuethread->join();
}

// Simulation wants data.
bool AllInOneControllerWrapper::atrias_gui_callback(atrias_controllers::atrias_srv::Request &req, atrias_controllers::atrias_srv::Response &res)
{
	int i;

	// Grab the request.
	this->controller_data->command = req.command;
	this->controller_data->controller_requested = req.controller_requested;

	// Clone data from the gui into the controller's data memory.
	for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
	{
		this->controller_data->data[i] = req.control_data[i];
	}

	// Pack the response.
	if ( this->controller_state->state == STATE_ENABLED )
	{
		res.status = CMD_RUN;
	}
	else
	{
		res.status = CMD_DISABLE;
	}

	res.time = Simulator::Instance()->GetSimTime().Double();
	res.body_angle = this->controller_input->body_angle;
	res.motor_angleA = this->controller_input->motor_angleA;
	res.motor_angleB = this->controller_input->motor_angleB;
	res.leg_angleA = this->controller_input->leg_angleA;
	res.leg_angleB = this->controller_input->leg_angleB;
	res.motor_torqueA = this->controller_output->motor_torqueA;
	res.motor_torqueB = this->controller_output->motor_torqueB;
	res.hor_vel = this->controller_input->horizontal_velocity;
	res.height = this->controller_input->height;

	// Clone data from the gui into the controller's data memory.
	for (i = 0; i < SIZE_OF_CONTROLLER_STATE_DATA; i++)
	{
		res.control_state[i] = this->controller_state->data[i];
	}

	return true;
}

// Manage the callback thread.
void AllInOneControllerWrapper::QueueThread()
{
  static const double timeout = 0.01;

  while (nh->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    queue.callAvailable(ros::WallDuration(timeout));
  }
}
