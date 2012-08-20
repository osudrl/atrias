#include <atrias_sim/planar_constraint.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(PlanarConstraint)

PlanarConstraint::PlanarConstraint()
{
	KP = 100000;
	KD = 100;
}

void PlanarConstraint::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
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
		boost::bind(&PlanarConstraint::OnUpdate, this));

	desired_pose = this->link->GetWorldPose();
}


// Called by the world update start event
void PlanarConstraint::OnUpdate()
{
	force = (this->desired_pose.pos - this->link->GetWorldPose().pos) * KP - this->link->GetRelativeLinearVel() * KD;
	torque = (this->desired_pose.rot.GetAsEuler() - this->link->GetWorldPose().rot.GetAsEuler()) * KP - this->link->GetRelativeAngularVel() * KD;
	// Remove one constraint
	force = math::Vector3(force[0], force[1], 0.0); // x, y, z
	torque = math::Vector3(torque[0], torque[1], torque[2]); // roll, pitch, yaw

	// Set the torques
	this->link->SetForce(force);
	this->link->SetTorque(torque);
}
