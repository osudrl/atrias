// Andrew Peekema

#include <drl_plugins/invertedPendulumWithToe.h>


// Register the controller using the gazebo controller factory
GZ_REGISTER_DYNAMIC_CONTROLLER("invertedPendulumWithToe", InvertedPendulumWithToe);



////////////////////////////////////////////////////////////////////////////////
// Constructor
InvertedPendulumWithToe::InvertedPendulumWithToe(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("InvertedPendulum controller requires an Model as its parent");

  Param::Begin(&this->parameters);
  this->bodyName1P = new ParamT<std::string>("bodyName1","link", 1);
  this->bodyName2P = new ParamT<std::string>("bodyName2","link", 1);
  Param::End();

  // Parameters
  pi = 3.14159265;
  l0 = 1; 
  alpha = 73.1*pi/180;
  k = 25000;
  stance = 0;
  zVelMPrev = -1;
  touchDown = 1;
  

  // Open file for writing data
  file.open("/home/drl/GazeboInvertedPendulum.txt", ios::out);
  file << "xPos zPos stance\n";

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
InvertedPendulumWithToe::~InvertedPendulumWithToe()
{
  delete this->bodyName1P;
  delete this->bodyName2P;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller (Get references to bodies)
void InvertedPendulumWithToe::LoadChild(XMLConfigNode *node)
{
  this->bodyName1P->Load(node);
  this->bodyName1 = this->bodyName1P->GetValue();
  this->bodyName2P->Load(node);
  this->bodyName2 = this->bodyName2P->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName1)) == NULL)
    ROS_FATAL("gazebo controller (invertedPendulumWithToe) error: bodyName1: %s does not exist\n",bodyName1.c_str());
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName2)) == NULL)
    ROS_FATAL("gazebo controller (invertedPendulumWithToe) error: bodyName2: %s does not exist\n",bodyName2.c_str());

  this->body1 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName1));
  this->body2 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName2));

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR("gazebo controller (invertedPendulumWithToe) update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

  // Mass Initial Conditions (velocity)
  this->lock.lock();
  this->body1->SetLinearVel(Vector3(3.699, 0, 0));
  this->lock.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void InvertedPendulumWithToe::UpdateChild()
{
    this->lock.lock();
	//Use the command ROS_INFO for debugging
    //ROS_INFO("xPos = %f\n", xPos); 
    
    // Mass particle state
    zPosM = this->body1->GetWorldPose().pos.z;
    xPosM = this->body1->GetWorldPose().pos.x;
    zVelM = this->body1->GetWorldLinearVel().z;

    // Foot state
    zPosF = this->body2->GetWorldPose().pos.z;
    xPosF = this->body2->GetWorldPose().pos.x;
    zVelF = this->body2->GetWorldLinearVel().z;

    // Flight
	if ( stance == 0 ) {
        // Test stance conditions
        if (( sgn(zVelM) != sgn(zVelMPrev) ) && ( zPosM > l0*cos(alpha) ) ) {
            touchDown = 1;
        }
        if ( ( zVelM < 0 ) && ( zPosM <= l0*sin(alpha) ) && ( touchDown == 1 ) ) {
            xfp = xPosM + l0*cos(alpha);
            orig_pose = this->body2->GetWorldPose();
            pose = orig_pose;
            pose.pos.x = xfp;
            pose.pos.y = 0.0;
            pose.pos.z = 0.050;
            this->body2->SetWorldPose(pose);
            stance = 1;
        }
        else if ( zPosM < l0*sin(alpha) ) {
            ROS_ERROR("Unstable Solution");
        }
        // No force needed
		force_on_mass = Vector3(0., 0., 0.);
        force_on_toe = Vector3(0., 0., 0.);
	}
    // Stance
    else if ( stance == 1 ) {
        // Test flight conditions
        //l = sqrt(pow((xPosM-xfp),2) + pow((zPosM-0),2));
        l = sqrt(pow((xPosM-xPosF),2) + pow((zPosM-zPosF),2));
        if ( l >= l0 ) {
            stance = 0;
            touchDown = 0;
        }
        // Calculate the spring force
        //Fx = k*(l0-l)*(xPosM-xfp)/l;
        //Fz = k*(l0-l)*(zPosM-0)/l;
        Fx = k*(l0-l)*(xPosM-xPosF)/l;
        Fz = k*(l0-l)*(zPosM-zPosF)/l;
        force_on_mass = Vector3(Fx, 0., Fz);
        force_on_toe = Vector3(-Fx, 0., -Fz);
    }

    
    ROS_INFO("xPosM = %f, zPosM = %f, stance = %f, zVelM = %f, touchDown = %f", xPosM, zPosM, stance, zVelM, touchDown);

    // Write data to file
    file << xPosM << " " << zPosM << " " << stance << "\n";

    // Apply the force
    this->body1->SetForce(force_on_mass);
    this->body2->SetForce(force_on_toe);

    zVelMPrev = zVelM;

    this->lock.unlock();

}
