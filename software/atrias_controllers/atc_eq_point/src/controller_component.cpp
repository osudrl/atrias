/*! \FILE CONTROLLER_COMPONENT.CPp
 *  \author Daniel Renjewski	Nov. 5, 2012
 *  \to run with simulation
 *  \brief Orocos Component code for the atc_eq_point controller.
 */

#include <atc_eq_point/controller_component.h>

namespace atrias {
namespace controller {

ATCEqPoint::ATCEqPoint(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log"),
	guiDataIn("gui_data_in")
{
	this->provides("atc")
	->addOperation("runController", &ATCEqPoint::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	// Add properties
	this->addProperty("spg0Name", spg0Name);
	this->addProperty("spg1Name", spg1Name);
	this->addProperty("spg2Name", spg2Name);
	this->addProperty("spg3Name", spg3Name);
	this->addProperty("spg4Name", spg4Name);
	this->addProperty("spg5Name", spg5Name);
	this->addProperty("pd0Name",  pd0Name);
	this->addProperty("pd1Name",  pd1Name);
	this->addProperty("pd2Name",  pd2Name);
	this->addProperty("pd3Name",  pd3Name);
	this->addProperty("pd4Name",  pd4Name);
	this->addProperty("pd5Name",  pd5Name);
	this->addProperty("hip0Name", hip0Name);
	this->addProperty("hip1Name", hip1Name);

	// Gains for PD controllers. These are set in the configureHook.
	legP = 600;
	legD = 15;
	hipP = 150;
	hipD = 10;

	// For the GUI
	addEventPort(guiDataIn);
	pubTimer = new GuiPublishTimer(20);

	// Logging
	// Create a port
	addPort(logPort);
	// Buffer port so we capture all data.
	ConnPolicy policy = RTT::ConnPolicy::buffer(100000);
	// Transport type = ROS
	policy.transport = 3;
	// ROS topic name
	policy.name_id = "/" + name + "_log";
	// Construct the stream between the port and ROS topic
	logPort.createStream(policy);

	log(Info) << "[ATCMT] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCEqPoint::runController(atrias_msgs::robot_state rs) {
	// Do nothing unless told otherwise
	co.lLeg.motorCurrentA   = 0.0;
	co.lLeg.motorCurrentB   = 0.0;
	co.lLeg.motorCurrentHip = 0.0;
	co.rLeg.motorCurrentA   = 0.0;
	co.rLeg.motorCurrentB   = 0.0;
	co.rLeg.motorCurrentHip = 0.0;

	MotorState desiredLAState,
	           desiredLBState,
	           desiredLHState,
	           desiredRAState,
	           desiredRBState,
	           desiredRHState;

	// Only run the controller when we're enabled
	if ((rtOps::RtOpsState)rs.rtOpsState != rtOps::RtOpsState::ENABLED) {
		// Keep desired motor angles equal to the current motor angles so the
		// motors don't jump when the controller is enabled.
		desiredLAState.ang = rs.lLeg.halfA.motorAngle;
		desiredLBState.ang = rs.lLeg.halfB.motorAngle;
		desiredLHState.ang = rs.lLeg.hip.legBodyAngle;
		desiredRAState.ang = rs.rLeg.halfA.motorAngle;
		desiredRBState.ang = rs.rLeg.halfB.motorAngle;
		desiredRHState.ang = rs.rLeg.hip.legBodyAngle;

		// Reset smooth path generators. This is needed in case the user
		// disables the controller before disabling the demo, which is a
		// problem since the generators do not run when the controller is
		// disabled, which means they will finish only once the controller is
		// re-enabled, and that only with outdated data.
		spg0IsFinished = true;
		spg1IsFinished = true;
		spg2IsFinished = true;
		spg3IsFinished = true;
		spg4IsFinished = true;
		spg5IsFinished = true;

		// Set state to something other than 2.
		state = 0;

		return co;
	}

	// begin control code //

	// map GUI buttons for GC (Simulation)
	rGC = guiIn.gc_r;
	lGC = guiIn.gc_l;

	// read toeswitch from robot
	//rGC=rs.rLeg.toeSwitch;
	//lGC=rs.lLeg.toeSwitch;


	// initial state
	// Calculate motor angles
	MotorAngle leftMotorAngle  = legToMotorPos(guiIn.pea, guiIn.l_leg_st);
	MotorAngle rightMotorAngle = legToMotorPos(guiIn.aea, guiIn.l_leg_st);
	int initDur = 2.0;   // Duration of initialization sequence.


	// Smoothly initialize and set state to 2.
	if (state == 0) {
		if (spg0IsFinished && spg1IsFinished && spg2IsFinished &&
		    spg3IsFinished && spg4IsFinished && spg5IsFinished) {
			spg0Init(rs.lLeg.halfA.motorAngle, leftMotorAngle.A, initDur);
			spg1Init(rs.lLeg.halfB.motorAngle, leftMotorAngle.B, initDur);
			spg2Init(rs.lLeg.hip.legBodyAngle, rs.lLeg.hip.legBodyAngle, initDur);
			spg3Init(rs.rLeg.halfA.motorAngle, rightMotorAngle.A, initDur);
			spg4Init(rs.rLeg.halfB.motorAngle, rightMotorAngle.B, initDur);
			spg5Init(rs.rLeg.hip.legBodyAngle, rs.rLeg.hip.legBodyAngle, initDur);

		}

		if (!spg0IsFinished && !spg1IsFinished && !spg2IsFinished &&
		    !spg3IsFinished && !spg4IsFinished && !spg5IsFinished) {
			
			spg2SetTgt(hip0Controller((rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle) / 2.0, rs.position.boomAngle, 0));
			spg5SetTgt(hip1Controller((rs.rLeg.halfA.legAngle + rs.rLeg.halfB.legAngle) / 2.0, rs.position.boomAngle, 1));

			desiredLAState = spg0RunController();
			desiredLBState = spg1RunController();
			desiredLHState = spg2RunController();
			desiredRAState = spg3RunController();
			desiredRBState = spg4RunController();
			desiredRHState = spg5RunController();

			co.lLeg.motorCurrentA   = pd0Controller(desiredLAState.ang, rs.lLeg.halfA.motorAngle, desiredLAState.vel, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB   = pd1Controller(desiredLBState.ang, rs.lLeg.halfB.motorAngle, desiredLBState.vel, rs.lLeg.halfB.motorVelocity);
			co.lLeg.motorCurrentHip = pd2Controller(desiredLHState.ang, rs.lLeg.hip.legBodyAngle, desiredLHState.vel, rs.lLeg.hip.legBodyVelocity);
			co.rLeg.motorCurrentA   = pd3Controller(desiredRAState.ang, rs.rLeg.halfA.motorAngle, desiredRAState.vel, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB   = pd4Controller(desiredRBState.ang, rs.rLeg.halfB.motorAngle, desiredRBState.vel, rs.rLeg.halfB.motorVelocity);
			co.rLeg.motorCurrentHip = pd5Controller(desiredRHState.ang, rs.rLeg.hip.legBodyAngle, desiredRHState.vel, rs.rLeg.hip.legBodyVelocity);

			if (spg0IsFinished && spg1IsFinished && spg2IsFinished &&
			    spg3IsFinished && spg4IsFinished && spg5IsFinished) {
                printf("Startup is finished\n");
				state = 2;
				sw_flight = true;
				sw_stance = true;
                t=1;
                }
		}
         co.command = medulla_state_run;
		return co;
	}

	// Hip control.
	co.lLeg.motorCurrentHip = pd2Controller(hip0Controller((rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle) / 2.0, rs.position.boomAngle, 0), rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = pd5Controller(hip1Controller((rs.rLeg.halfA.legAngle + rs.rLeg.halfB.legAngle) / 2.0, rs.position.boomAngle, 1), rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

	// calculate leg angle and length

    amp=guiIn.l_leg_st-guiIn.l_leg_fl;
	phi_rLeg = (rs.rLeg.halfA.motorAngle+rs.rLeg.halfB.motorAngle)/2;		        //right leg angle
	phi_lLeg = (rs.lLeg.halfA.motorAngle+rs.lLeg.halfB.motorAngle)/2;		        //left leg angle
	phiAf_des = guiIn.aea - acos (guiIn.l_leg_st);							//desired motor position for flight MOTOR A
	phiBs_des = guiIn.pea + acos (guiIn.l_leg_st);							//desired motor position for stance MOTOR B
	logData.state = state;
	max_phi_swing = guiIn.aea-(guiIn.pea-guiIn.aea)*guiIn.d_as;
	
	

// s - stance leg status
// t - flight leg timing
   
switch (state)
{
    case 1:
        s = 1 - (guiIn.pea-phi_rLeg) / (guiIn.pea - guiIn.aea);
		break;
    case 2:
        s =1 - (guiIn.pea-phi_lLeg) / (guiIn.pea - guiIn.aea);
        break;
    default:
        break;
}
time++;
t = time / guiIn.p_af;
if (t > 1)
{
	t=1;
}

switch (guiIn.control)
{
case 1:
	if (state == 2 && phi_rLeg<M_PI/2 && rGC)								//switch left / right stepping 
	{
		state = 1;
		sw_stance = false;
		sw_flight = false;
        s=0;
        t=0;
		time=0;
	}
	if (state == 1 && phi_lLeg < M_PI/2 && lGC) 	
	{
		state = 2;
		sw_stance = false;
		sw_flight = false;
        s=0;
        t=0;
		time=0;
	}
	break;
case 2:
  	// state machine
	//if (state == 2 && phi_rLeg<M_PI/2 && rGC)								//switch left / right stepping automatically
	    if ((state == 2) && (s>0.99))
	{
		state = 1;
		sw_stance = false;
		sw_flight = false;
        s=0;
        t=0;
		time=0;
	}
	//if (state == 1 && phi_lLeg < M_PI/2 && lGC) 	
    if ((state == 1) && (s>0.99))
	{
		state = 2;
		sw_stance = false;
		sw_flight = false;
        s=0;
        t=0;
		time=0;
	}
	break;
default:
	break;
}

printf("s: %f t: %f time: %i \n",s,t,time);
// command legs ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//************************************************************stance control right leg***************************************************************************************************************************
	switch (state)																															//stance leg right, flight leg left
	{
	case 1:
		if ((rs.rLeg.halfB.motorAngle < phiBs_des) && !sw_stance)
		{																								// stance leg rotate to pea
					// asymmetry - extend right leg
			        rightMotorAngle = legToMotorPos(phi_rLeg,guiIn.l_leg_st);
					//##rightMotorAngle = legToMotorPos(phi_rLeg,guiIn.l_leg_st);
					D4.set(guiIn.d_ls);
					P4.set(guiIn.p_ls);
					co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0.5,rs.rLeg.halfB.motorVelocity) + guiIn.l_st;
					D3.set(guiIn.d_ls);
					P3.set(guiIn.p_ls);
					co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0.5,rs.rLeg.halfA.motorVelocity);
					//logData.currState = 1;
		} else 
		{																															 // if pea was reached once
			sw_stance=true;
			rightMotorAngle = legToMotorPos(guiIn.pea, guiIn.l_leg_st);
			D4.set(guiIn.d_ls);
			P4.set(guiIn.p_ls);
			co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity);
			D3.set(guiIn.d_ls);
			P3.set(guiIn.p_ls);
			co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
			//logData.currState = 2;
		}
		
	//****************************************************************flight control left leg*************************************************************************************************************
		if ((t < 1) && (!sw_flight)) 
        {
						if (t<guiIn.l_fl){
							phi_lLeg=guiIn.pea - ((t / guiIn.l_fl) * (guiIn.pea-max_phi_swing));
							P0.set(guiIn.p_lf);
							P1.set(guiIn.p_lf);
						} else {
							phi_lLeg=max_phi_swing + ( (t-guiIn.l_fl)/(1-guiIn.l_fl) * (guiIn.aea - max_phi_swing));
							P0.set(guiIn.p_lf + ((t-guiIn.l_fl)/(1-guiIn.l_fl) * (guiIn.p_ls - guiIn.p_lf)));
							P1.set(guiIn.p_lf + ((t-guiIn.l_fl)/(1-guiIn.l_fl) * (guiIn.p_ls - guiIn.p_lf)));
						}
						//map leg angle sweep of flight leg to 0->1
						//keep desired leg length -> shorten leg depending on leg position
						l_swing = sin (t*M_PI) * (-amp) + guiIn.l_leg_st;
						//##l_swing = sin (-M_PI/2 + 2 * M_PI * t) * (-amp/2) + guiIn.l_leg_fl + (amp / 2);
						//printf("left! t: %f, phi_min: %f, swing_leg angle: %f, l_swing: %f\n",t,max_phi_swing,phi_lLeg,l_swing);
                        leftMotorAngle = legToMotorPos(phi_lLeg,l_swing);
                        //printf("s: %f l_des: %f phi_des: %f phi_rB: %f\n",s,l_swing,leftMotorAngle.A,rs.rLeg.halfB.motorAngle);
						D0.set(guiIn.d_lf);
						//P0.set(guiIn.p_lf);
						co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
						D1.set(guiIn.d_lf);
						//P1.set(guiIn.p_lf);
						co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity);
						//logData.flightState = 1;
		} else 
		{																														// aea is reached once
			sw_flight=true;
			leftMotorAngle = legToMotorPos(guiIn.aea,guiIn.l_leg_st);
			D0.set(guiIn.d_ls);
			P0.set(guiIn.p_ls);
			co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
			D1.set(guiIn.d_ls);
			P1.set(guiIn.p_ls);
			co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity)+2;
			//logData.flightState = 2;
		}
		break;


    //********************************************************************************************************************************************************************************
	case 2:                         // stance leg left, swing leg right
		if ((rs.lLeg.halfB.motorAngle < phiBs_des) && !sw_stance) 
		{           // stance leg rotate to pea
					// asymmetry - extend left leg
					//##leftMotorAngle = legToMotorPos(phi_lLeg,(guiIn.l_leg_st*cos(M_PI/2 - guiIn.aea))/cos(M_PI/2-phi_lLeg));
					leftMotorAngle = legToMotorPos(phi_lLeg,guiIn.l_leg_st);
					D1.set(guiIn.d_ls);
					P1.set(guiIn.p_ls);
					co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0.5,rs.lLeg.halfB.motorVelocity)  + guiIn.l_st;
					D0.set(guiIn.d_ls);
					P0.set(guiIn.p_ls);
					co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0.5,rs.lLeg.halfA.motorVelocity);
					//logData.currState = 3;
		} else 
		{                        // if aea was reached once
			sw_stance=true;
			leftMotorAngle = legToMotorPos(guiIn.pea,guiIn.l_leg_st);
			D1.set(guiIn.d_ls);
			P1.set(guiIn.p_ls); 
			co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity);
			D0.set(guiIn.d_ls);
			P0.set(guiIn.p_ls); 
		    co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
			//logData.currState = 4;
		  }
		
	//*******************************************************************************************************************************************************************************************************************
		if ((t<1) && (!sw_flight))
        {
        				//map leg angle sweep of flight leg to 0->1
						//keep desired leg length -> shorten leg depending on leg position
						if (t<guiIn.l_fl){
							phi_rLeg=guiIn.pea - (t / guiIn.l_fl) * (guiIn.pea-max_phi_swing);
							P3.set(guiIn.p_lf);
							P4.set(guiIn.p_lf);
						} else {
							phi_rLeg=max_phi_swing + ( (t-guiIn.l_fl)/(1-guiIn.l_fl) * (guiIn.aea - max_phi_swing));
							P3.set(guiIn.p_lf + ((t-guiIn.l_fl)/(1-guiIn.l_fl) * (guiIn.p_ls - guiIn.p_lf)));
							P4.set(guiIn.p_lf + ((t-guiIn.l_fl)/(1-guiIn.l_fl) * (guiIn.p_ls - guiIn.p_lf)));
							//printf("P0: %f\n",guiIn.p_lf + ((t-guiIn.l_fl)/(1-guiIn.l_fl) * (guiIn.p_ls - guiIn.p_lf)));
						}
						//keep desired leg length -> shorten leg depending on leg position
						l_swing = sin (t*M_PI) * (-amp) + guiIn.l_leg_st;
						//printf("right! t: %f, phi_min: %f, swing_leg angle: %f, l_swing: %f\n",t,max_phi_swing,phi_lLeg,l_swing);
						//l_swing = sin (M_PI * s) * (-amp) + guiIn.l_leg_fl;
						//phi_rLeg=guiIn.pea-s*(guiIn.pea-guiIn.aea);
                        rightMotorAngle = legToMotorPos(phi_rLeg,l_swing);
						D3.set(guiIn.d_lf);
						//P3.set(guiIn.p_lf);
						co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
						D4.set(guiIn.d_lf);
						//P4.set(guiIn.p_lf);
						co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity);
						//logData.flightState = 3;
		} else 
		{ // aea is reached once
			sw_flight=true;
			rightMotorAngle = legToMotorPos(guiIn.aea, guiIn.l_leg_st);
			D3.set(guiIn.d_ls);
			P3.set(guiIn.p_ls);
			co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
			D4.set(guiIn.d_ls);
			P4.set(guiIn.p_ls);
			co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity)+2;
			//logData.flightState = 4;
		}
		break;

	default:
		break;
	} // end switch

	// Stuff the msg
	//co.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
	//co.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
	//co.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

	// end control code //

	// Command a run state
	co.command = medulla_state_run;

	// Stuff the msg and push to ROS for logging
	logPort.write(logData);

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCEqPoint::configureHook() {
	// Connect to the subcontrollers
	// Get references to subcontroller component properties
	spg0 = this->getPeer(spg0Name);
	if (spg0) {
		spg0Init          = spg0->provides("smoothPath")->getOperation("init");
		spg0RunController = spg0->provides("smoothPath")->getOperation("runController");
		spg0IsFinished    = spg0->properties()->getProperty("isFinished");
	}

	spg1 = this->getPeer(spg1Name);
	if (spg1) {
		spg1Init          = spg1->provides("smoothPath")->getOperation("init");
		spg1RunController = spg1->provides("smoothPath")->getOperation("runController");
		spg1IsFinished    = spg1->properties()->getProperty("isFinished");
	}

	spg2 = this->getPeer(spg2Name);
	if (spg2) {
		spg2Init          = spg2->provides("smoothPath")->getOperation("init");
		spg2RunController = spg2->provides("smoothPath")->getOperation("runController");
		spg2SetTgt        = spg2->provides("smoothPath")->getOperation("setTgt");
		spg2IsFinished    = spg2->properties()->getProperty("isFinished");
	}

	spg3 = this->getPeer(spg3Name);
	if (spg3) {
		spg3Init          = spg3->provides("smoothPath")->getOperation("init");
		spg3RunController = spg3->provides("smoothPath")->getOperation("runController");
		spg3IsFinished    = spg3->properties()->getProperty("isFinished");
	}

	spg4 = this->getPeer(spg4Name);
	if (spg4) {
		spg4Init          = spg4->provides("smoothPath")->getOperation("init");
		spg4RunController = spg4->provides("smoothPath")->getOperation("runController");
		spg4IsFinished    = spg4->properties()->getProperty("isFinished");
	}

	spg5 = this->getPeer(spg5Name);
	if (spg5) {
		spg5Init          = spg5->provides("smoothPath")->getOperation("init");
		spg5RunController = spg5->provides("smoothPath")->getOperation("runController");
		spg5SetTgt        = spg5->provides("smoothPath")->getOperation("setTgt");
		spg5IsFinished    = spg5->properties()->getProperty("isFinished");
	}

	pd0 = this->getPeer(pd0Name);
	if (pd0)
		pd0Controller = pd0->provides("pd")->getOperation("runController");

	pd1 = this->getPeer(pd1Name);
	if (pd1)
		pd1Controller = pd1->provides("pd")->getOperation("runController");

	pd2 = this->getPeer(pd2Name);
	if (pd2)
		pd2Controller = pd2->provides("pd")->getOperation("runController");

	pd3 = this->getPeer(pd3Name);
	if (pd3)
		pd3Controller = pd3->provides("pd")->getOperation("runController");

	pd4 = this->getPeer(pd4Name);
	if (pd4)
		pd4Controller = pd4->provides("pd")->getOperation("runController");

	pd5 = this->getPeer(pd5Name);
	if (pd5)
		pd5Controller = pd5->provides("pd")->getOperation("runController");
	
	hip0 = this->getPeer(hip0Name);
	if (hip0)
		hip0Controller = hip0->provides("hipAngle")->getOperation("runController");
	
	hip1 = this->getPeer(hip1Name);
	if (hip1)
		hip1Controller = hip1->provides("hipAngle")->getOperation("runController");


	P0 = pd0->properties()->getProperty("P");
	D0 = pd0->properties()->getProperty("D");
	P1 = pd1->properties()->getProperty("P");
	D1 = pd1->properties()->getProperty("D");
	P2 = pd2->properties()->getProperty("P");
	D2 = pd2->properties()->getProperty("D");
	P3 = pd3->properties()->getProperty("P");
	D3 = pd3->properties()->getProperty("D");
	P4 = pd4->properties()->getProperty("P");
	D4 = pd4->properties()->getProperty("D");
	P5 = pd5->properties()->getProperty("P");
	D5 = pd5->properties()->getProperty("D");

	P0.set(legP);
	D0.set(legD);
	P1.set(legP);
	D1.set(legD);
	P2.set(hipP);
	D2.set(hipD);
	P3.set(legP);
	D3.set(legD);
	P4.set(legP);
	D4.set(legD);
	P5.set(hipP);
	D5.set(hipD);


	legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

	log(Info) << "[ATCMT] configured!" << endlog();
	return true;
}

bool ATCEqPoint::startHook() {
	log(Info) << "[ATCMT] started!" << endlog();
	return true;
}

void ATCEqPoint::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCEqPoint::stopHook() {
	log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCEqPoint::cleanupHook() {
	log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCEqPoint)

}
}

// vim: noexpandtab

