#include <atrias_sim/pause_world.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(PauseWorld)

PauseWorld::PauseWorld()
{
	count = 0;
}

void PauseWorld::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// Pointer to the world
	this->world = _parent;

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateStart(
		boost::bind(&PauseWorld::OnUpdate, this));

}


// Called by the world update start event
void PauseWorld::OnUpdate()
{
	// Pause the world
	if (count == 0)
	{
		pause_world(this->world, true);
		count++;
	}
}
