#include <atrias_sim/ground_contact.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ModelPush)

ModelPush::ModelPush()
{
	this->force_1 = 10;
	this->force_2 = 10;
	this->force_3 = 9;
}

void ModelPush::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// Pointer to the world
	this->world = _parent;

	// Pointer to the model
	this->model_1 = this->world->GetModel("model_1");
	this->model_2 = this->world->GetModel("model_2");
	this->model_3 = this->world->GetModel("model_3");

	// Pointer to the link
	this->link_1 = this->model_1->GetLink("link_1");
	this->link_2 = this->model_2->GetLink("link_2");
	this->link_3 = this->model_3->GetLink("link_3");

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateStart(
		boost::bind(&ModelPush::OnUpdate, this));

	forceVec_1 = math::Vector3(0, force_1, 0);
	forceVec_2 = math::Vector3(0, force_2, 0);
	forceVec_3 = math::Vector3(0, force_3, 0);
}


// Called by the world update start event
void ModelPush::OnUpdate()
{
	this->link_1->AddForce(forceVec_1);
	this->link_2->AddForce(forceVec_2);
	this->link_3->AddForce(forceVec_3);
}
