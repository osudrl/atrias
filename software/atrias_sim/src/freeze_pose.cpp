// Author: Andrew Peekema

#include <atrias_sim/freeze_pose.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(FreezePose)

FreezePose::FreezePose()
{
    KP = 100000;
    KD = 100;
}

void FreezePose::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Pointer to the world
    this->world = _parent;

    // Pointer to the model
    if (_sdf->HasElement("modelName"))
    {
        this->modelName = _sdf->GetElement("modelName")->GetValueString();
        this->model = this->world->GetModel(this->modelName);
    }
    else
        gzerr << "Gazebo freezePose plugin missing valid modelName\n";

    // Pointer to the link
    if (_sdf->HasElement("linkName"))
    {
        this->linkName = _sdf->GetElement("linkName")->GetValueString();
        this->link = this->model->GetLink(this->linkName);
    }
    else
        gzerr << "Gazebo freezePose plugin missing valid linkName\n";

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateStart(
        boost::bind(&FreezePose::OnUpdate, this));

    desired_pose = this->link->GetWorldPose();
}


// Called by the world update start event
void FreezePose::OnUpdate()
{
    force = (this->desired_pose.pos - this->link->GetWorldPose().pos) * KP - this->link->GetRelativeLinearVel() * KD;
    this->link->SetForce(force);

    this->link->SetTorque((this->desired_pose.rot.GetAsEuler() - this->link->GetWorldPose().rot.GetAsEuler()) * KP - this->link->GetRelativeAngularVel() * KD);
}
