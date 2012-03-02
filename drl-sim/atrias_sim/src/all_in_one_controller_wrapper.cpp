// Devin Koepl

#include <atrias_sim/all_in_one_controller_wrapper.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("all_in_one_controller_wrapper", AllInOneControllerWrapper);

////////////////////////////////////////////////////////////////////////////////
// Constructor

AllInOneControllerWrapper::AllInOneControllerWrapper(Entity *parent)
: Controller(parent)
{
    //	PhysicsEngine::InitForThread();

    this->myParent = dynamic_cast<Model*> (this->parent);

    if (!this->myParent)
        gzthrow("ControllerWrapper requires a Model as its parent");

    Param::Begin(&this->parameters);
    this->bodyNameP = new ParamT<std::string > ("bodyName", "link", 1);
    this->motorANameP = new ParamT<std::string > ("motorAName", "link", 1);
    this->motorBNameP = new ParamT<std::string > ("motorBName", "link", 1);
    this->legANameP = new ParamT<std::string > ("legAName", "link", 1);
    this->legBNameP = new ParamT<std::string > ("legBName", "link", 1);
    Param::End();

    this->controller_input = new ControllerInput();
    this->controller_output = new ControllerOutput();
    this->controller_state = new ControllerState();
    this->controller_data = new ControllerData();

    this->controller_state->state = CSSM_STATE_INIT;

    atrias_data_publish_counter = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor

AllInOneControllerWrapper::~AllInOneControllerWrapper()
{
    controller_state->state = CSSM_STATE_FINI;
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
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->bodyName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: bodyName: %s does not exist\n", bodyName.c_str());
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->motorAName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: motorAName: %s does not exist\n", motorAName.c_str());
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->motorBName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: motorBName: %s does not exist\n", motorBName.c_str());
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->legAName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: legAName: %s does not exist\n", legAName.c_str());
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->legBName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: legBName: %s does not exist\n", legBName.c_str());

    this->body = dynamic_cast<Body*> (this->myParent->GetBody(bodyName));
    this->motorA = dynamic_cast<Body*> (this->myParent->GetBody(motorAName));
    this->motorB = dynamic_cast<Body*> (this->myParent->GetBody(motorBName));
    this->legA = dynamic_cast<Body*> (this->myParent->GetBody(legAName));
    this->legB = dynamic_cast<Body*> (this->myParent->GetBody(legBName));

    // check update rate against world physics update rate
    // should be equal or higher to guarantee the wrench applied is not "diluted"
    if (this->updatePeriod > 0 &&
            (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0 / this->updatePeriod))
        ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

    this->generate_controller_input();
    control_switcher_state_machine(controller_input, controller_output, controller_state, controller_data);
    int argc = 0;
    char** argv = NULL;
    //ros::init(argc, argv, "gui_interface", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    ros::init(argc, argv, "gui_interface");
    ros::NodeHandle nh;

    atrias_sim_sub = nh.subscribe("atrias_controller_requests", 0, &AllInOneControllerWrapper::atrias_gui_callback, this);
    atrias_sim_pub = nh.advertise<atrias_msgs::atrias_data>("atrias_data_50_hz", 10);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller

void AllInOneControllerWrapper::UpdateChild()
{
    this->lock.lock();

    this->generate_controller_input();

    this->poke_ros();

    control_switcher_state_machine(this->controller_input, this->controller_output, this->controller_state, this->controller_data);

    this->controller_output->motor_torqueA = CLAMP(this->controller_output->motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ);
    this->controller_output->motor_torqueB = CLAMP(this->controller_output->motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ);

    this->motorA->SetTorque(Vector3(0., -this->controller_output->motor_torqueA * GEAR_RATIO, 0.));
    this->motorB->SetTorque(Vector3(0., -this->controller_output->motor_torqueB * GEAR_RATIO, 0.));

    this->lock.unlock();
}

void AllInOneControllerWrapper::poke_ros() {
    ros::spinOnce();

    if (atrias_data_publish_counter % 20 == 0) {   // 50 Hz publish rate.
        if (this->controller_state->state == CSSM_STATE_ENABLED)
        {
            ad.status = CMD_RUN;
        }
        else
        {
            ad.status = CMD_DISABLE;
        }

        ad.time = Simulator::Instance()->GetSimTime().Double();
        ad.body_angle = this->controller_input->body_angle;
        ad.motor_angleA = this->controller_input->motor_angleA;
        ad.motor_angleB = this->controller_input->motor_angleB;
        ad.leg_angleA = this->controller_input->leg_angleA;
        ad.leg_angleB = this->controller_input->leg_angleB;
        ad.motor_torqueA = this->controller_output->motor_torqueA;
        ad.motor_torqueB = this->controller_output->motor_torqueB;
        ad.xPosition = this->controller_input->xPosition;
        ad.yPosition = this->controller_input->yPosition;
        ad.zPosition = this->controller_input->zPosition;
        ad.xVelocity = this->controller_input->xVelocity;
        ad.yVelocity = this->controller_input->yVelocity;
        ad.zVelocity = this->controller_input->zVelocity;

        // Clone data from the gui into the controller's data memory.
        for (i = 0; i < SIZE_OF_CONTROLLER_STATE_DATA; i++)
        {
            ad.control_state[i] = this->controller_state->data[i];
        }

        atrias_sim_pub.publish(ad);
        atrias_data_publish_counter = (atrias_data_publish_counter + 1) % 1000;
    }
}

void AllInOneControllerWrapper::generate_controller_input()
{
    double angle;
    Vector3 axis;

    this->body->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controller_input->body_angle = angle * axis.y;
    this->motorA->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controller_input->motor_angleA = -angle * axis.y + 3. * PI / 4. - PI;
    //ROS_INFO("maA = %.3f", angle);
    this->motorB->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controller_input->motor_angleB = -angle * axis.y + PI / 4. + PI;
    //ROS_INFO("maB = %.3f", angle);
    this->legA->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controller_input->leg_angleA = -angle * axis.y + 3. * PI / 4. - PI;
    //ROS_INFO("laA = %.3f", angle);
    this->legB->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controller_input->leg_angleB = -angle * axis.y + PI / 4. + PI;
    //ROS_INFO("laB = %.3f\n\n", angle);

    this->controller_input->xPosition = this->body->GetWorldPose().pos.x;
    this->controller_input->yPosition = this->body->GetWorldPose().pos.y;
    this->controller_input->zPosition = this->body->GetWorldPose().pos.z;

    this->controller_input->xVelocity = this->body->GetWorldLinearVel().x;
    this->controller_input->yVelocity = this->body->GetWorldLinearVel().y;
    this->controller_input->zVelocity = this->body->GetWorldLinearVel().z;

    this->controller_input->motor_velocityA = -this->motorA->GetWorldAngularVel().y;
    this->controller_input->motor_velocityB = -this->motorB->GetWorldAngularVel().y;
    this->controller_input->leg_velocityA = -this->legA->GetWorldAngularVel().y;
    this->controller_input->leg_velocityB = -this->legB->GetWorldAngularVel().y;
}

void AllInOneControllerWrapper::atrias_gui_callback(const atrias_msgs::atrias_controller_requests &cr)
{
    // Grab the request.
    this->controller_data->command = cr.command;
    this->controller_data->controller_requested = cr.controller_requested;

    // Clone data from the gui into the controller's data memory.
    for (int i=0; i<SIZE_OF_CONTROLLER_DATA; i++)
    {
        this->controller_data->data[i] = cr.control_data[i];
    }
}

