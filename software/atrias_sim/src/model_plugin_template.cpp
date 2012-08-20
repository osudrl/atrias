#include <atrias_sim/model_plugin_template.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)

ModelPush::ModelPush()
{
	this->force = 0.3;
}

void ModelPush::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	// Store the pointer to the model
	this->model = _parent;

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
