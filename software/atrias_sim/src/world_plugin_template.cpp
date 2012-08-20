#include <atrias_sim/world_plugin_template.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ModelPush)

// Constructor
ModelPush::ModelPush()
{
	force = 10;
}

void ModelPush::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// Pointer to the world
	this->world = _parent;

	// Pointer to the model
	this->model = this->world->GetModel("model_name");

	// Pointer to the link
	this->link = this->model->GetLink("link_name");

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateStart(
		boost::bind(&ModelPush::OnUpdate, this));

	forceVec = math::Vector3(0, force, 0);
}


// Called by the world update start event
void ModelPush::OnUpdate()
{
	this->link->AddForce(forceVec);
}
