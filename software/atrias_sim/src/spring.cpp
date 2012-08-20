// Author: Andrew Peekema

#include <atrias_sim/spring.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Spring)

Spring::Spring()
{
	// Nothing
}

void Spring::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	// Pointer to the model
	this->model = _parent;

	// Pointer to link1
	if (_sdf->HasElement("link1Name"))
	{
		this->link1Name = _sdf->GetElement("link1Name")->GetValueString();
		this->link1 = this->model->GetLink(this->link1Name);
	}
	else
		gzerr << "Gazebo Spring plugin missing valid link1Name\n";

	// Pointer to link2
	if (_sdf->HasElement("link2Name"))
	{
		this->link2Name = _sdf->GetElement("link2Name")->GetValueString();
		this->link2 = this->model->GetLink(this->link2Name);
	}
	else
		gzerr << "Gazebo Spring plugin missing valid link2Name\n";

	// Get the rotational stiffness
	if (_sdf->HasElement("rotationalStiffness"))
	{
		rotationalStiffness = _sdf->GetElement("rotationalStiffness")->GetValueDouble();
	}
	else
		gzerr << "Gazebo Spring plugin missing valid rotationalStiffness\n";

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateStart(
		boost::bind(&Spring::OnUpdate, this));
}


// Called by the world update start event
void Spring::OnUpdate()
{
	// Calculate the relative angular deflection and torque
	this->link1->GetRelativePose().rot.GetAsAxis(axis, angle);
	angle1 = angle*axis.y + M_PI/4.;
	// Keep the angle between 2*M_PI and -2*M_PI
	if (angle1 < 0)
		angle1 = -1.0*fmod(-1.0*angle1, 2*M_PI);
	else if (angle1 > 0)
		angle1 = fmod(angle1, 2*M_PI);

	this->link2->GetRelativePose().rot.GetAsAxis(axis, angle);
	angle2 = angle*axis.y + M_PI/4.;
	if (angle2 < 0)
		angle2 = -1.0*fmod(-1.0*angle2, 2*M_PI);
	else if (angle2 > 0)
		angle2 = fmod(angle2, 2*M_PI);

	// Find the angular deflection
	diff = angle2 - angle1;
	if (diff < -M_PI)
		diff = angle2 - angle1 + 2*M_PI;
	else if (diff > M_PI)
		diff = angle2 - angle1 - 2*M_PI;

	// Convert the angular deflection to a torque
	torque = diff * rotationalStiffness;

	torque_vec = math::Vector3(0., torque, 0.);

	// Apply the torques
	this->link1->AddRelativeTorque(torque_vec);
	this->link2->AddRelativeTorque(torque_vec * -1.);
}
