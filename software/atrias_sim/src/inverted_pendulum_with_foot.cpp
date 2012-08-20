// Author: Andrew Peekema

#include <atrias_sim/inverted_pendulum_with_foot.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(InvertedPendulum)

// Constructor
InvertedPendulum::InvertedPendulum()
{
	// Parameters
	pi = 3.14159265;
	l0 = 1;
	alpha = 73.1*pi/180;
	k = 25000;
	stance = 0;
	touchDown = 1;
	count = 0;
	footOffset = 0.05;

	// Open file for writing data                                                
	const char *homedir = getenv("HOME");
	char file_location[128];
	strcpy (file_location, homedir);
	strcat (file_location, "/GazeboInvertedPendulum.txt");
	file.open(file_location, std::ios::out);
	file << "xPos zPos stance\n";
	// Let the user know a logfile was created
	gzdbg << "Plugin logfile opened at: " << file_location << "\n";
}

// Initial plugin load
void InvertedPendulum::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// Pointer to the world
	this->world = _parent;

	// Pointer to the model
	this->model = this->world->GetModel("point_mass_model");
	this->foot_model = this->world->GetModel("foot_model");

	// Pointer to the link
	this->link = this->model->GetLink("point_mass_link");
	this->foot_link = this->foot_model->GetLink("foot_link");

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateStart(
		boost::bind(&InvertedPendulum::OnUpdate, this));

	// Initial Condition (Height)
	math::Pose orig_pose = this->link->GetWorldPose();
	math::Pose pose = orig_pose;
	pose.pos.z = 0.959+footOffset;
	this->link->SetWorldPose(pose);

}


// Called by the world update start event
void InvertedPendulum::OnUpdate()
{
	// Setup
	// Note: For some reason gazebo only updates the velocity and accepts the pause_world command in this loop
	if (count == 0)
	{
		// Pause the world
		pause_world(this->world, true);
		this->link->SetLinearVel(math::Vector3(3.699, 0, 0));
		count++;
	}

	// State
	zPos = this->link->GetWorldPose().pos.z;                                  
	xPos = this->link->GetWorldPose().pos.x;                                  
	zVel = this->link->GetWorldLinearVel().z;                                 
	footX = this->foot_link->GetWorldPose().pos.x;
	footZ = this->foot_link->GetWorldPose().pos.z;

	// Flight
	if ( stance == 0 )
	{
			// Touchdown conditions
			if (( zVel < 0 ) && ( zPos > (l0*cos(alpha)+footOffset) ) )
				touchDown = 1;
			// Stance Conditions
			if ( ( zVel < 0 ) && ( zPos <= (l0*sin(alpha)+footOffset) ) && ( touchDown == 1 ) )
			{
				xfp = xPos + l0*cos(alpha);
				stance = 1;
				// Move the foot
				foot_pose.pos.x = xfp;
				foot_pose.pos.y = 0;
				foot_pose.pos.z = footOffset;
				foot_pose.rot.SetFromEuler(math::Vector3(0,0,0));
				this->foot_model->SetWorldPose(foot_pose);
			}
			// Else we're not taking off
			else if ( zPos < (l0*sin(alpha)+footOffset) )
				gzerr << "Unstable Solution\n";

			// No force applied during flight
			massForceVec = math::Vector3(0., 0., 0.);
			footForceVec = math::Vector3(0., 0., 0.);
	}
	// Stance
	else if ( stance == 1 )
	{
		// Flight conditions
		l = sqrt(pow((xPos-xfp),2) + pow((zPos-footZ),2));
		if ( l >= l0 )
		{
			stance = 0;
			touchDown = 0;
		}
		// Calculate the spring force
		massForceVec = math::Vector3(k*(l0-l)*(xPos-footX)/l, 0., k*(l0-l)*(zPos-footZ)/l);
		footForceVec = math::Vector3(-massForceVec.x, 0., -massForceVec.z);
	}

	gzdbg << "xpos = " << xPos << ", zPos = " << zPos << ", stance = " << stance << ", zVel = " << zVel << ", touchDown = " << touchDown << "\n";

	// Write data to file
	file << xPos << " " << zPos << " " << stance << "\n";

	// Apply the force
	this->link->AddForce(massForceVec);
	this->foot_link->AddForce(footForceVec);

}
