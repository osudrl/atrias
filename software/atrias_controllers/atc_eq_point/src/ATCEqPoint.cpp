/**
  * @file ATCEqPoint.cpp
  * @author Daniel Renjewski
  * @brief Code for the atc_eq_point controller.
  */

#include "atc_eq_point/ATCEqPoint.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCEqPoint::ATCEqPoint(string name) :
	ATC(name),
	commonToolkit(this, "commonToolkit"),
	hipKine(this, "hipKine"),
	pd0Controller(this, "pd0Controller"),
	pd1Controller(this, "pd1Controller"),
	pd2Controller(this, "pd2Controller"),
	pd3Controller(this, "pd3Controller"),
	pd4Controller(this, "pd4Controller"),
	pd5Controller(this, "pd5Controller")
{
	// Gains for PD controllers.
	legP = 600;
	legD = 15;
	hipP = 150;
	hipD = 10;

	// Initialize gains
	pd0Controller.P = legP;
	pd1Controller.P = legP;
	pd2Controller.P = hipP;
	pd3Controller.P = legP;
	pd4Controller.P = legP;
	pd5Controller.P = hipP;
	pd0Controller.D = legD;
	pd1Controller.D = legD;
	pd2Controller.D = hipD;
	pd3Controller.D = legD;
	pd4Controller.D = legD;
	pd5Controller.D = hipD;

	// Initialize state
	state = 0;

	// Used the generic startup controller
	setStartupEnabled(true);
}

void ATCEqPoint::controller() {
	MotorState desiredLAState,
			 desiredLBState,
			 desiredRAState,
			 desiredRBState;

	// Handle idle mode
	if (!isEnabled()) {
		// We're disabled... leave idle mode
		idle_mode = false;
	}
	if (guiIn.control == 0) {
		// GUI is commanding idle mode
		idle_mode = true;
	}

	// Run the idle mode controller
	if (idle_mode) {
		// Set the hip controller P gains to 0 for a gentle relaxation.
		pd2Controller.P = (0.0);
		pd5Controller.P = (0.0);
		co.lLeg.motorCurrentHip = pd2Controller(0.0, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
		co.rLeg.motorCurrentHip = pd5Controller(0.0, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

		return;
	} else {
		// Revert the hip gains
		pd2Controller.P = hipP;
		pd5Controller.P = hipP;
	}

	// Only run the controller when we're enabled
	if (!isEnabled()) {
		return;
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
	MotorAngle leftMotorAngle;
	std::tie(leftMotorAngle.A,  leftMotorAngle.B)  = commonToolkit.legPos2MotorPos(guiIn.pea, guiIn.l_leg_st);
	MotorAngle rightMotorAngle;
	std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(guiIn.aea, guiIn.l_leg_st);

	// Hip control.
	LeftRight hipTgts;
	LeftRight toePosition;
	toePosition.left  = guiIn.lhip_pos;
	toePosition.right = guiIn.rhip_pos;;
	std::tie(hipTgts.left, hipTgts.right) = hipKine.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
	co.lLeg.motorCurrentHip = pd2Controller(hipTgts.left,  rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = pd5Controller(hipTgts.right, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

	// Smoothly initialize and set state to 2.
	if (state == 0) {
		// Desired leg motor positions
		desiredLAState.ang = leftMotorAngle.A;
		desiredLAState.vel = 0.0;
		desiredLBState.ang = leftMotorAngle.B;
		desiredLBState.vel = 0.0;
		desiredRAState.ang = rightMotorAngle.A;
		desiredRAState.vel = 0.0;
		desiredRBState.ang = rightMotorAngle.B;
		desiredRBState.vel = 0.0;

		// Run the PD controllers
		co.lLeg.motorCurrentA = pd0Controller(desiredLAState.ang, rs.lLeg.halfA.motorAngle, desiredLAState.vel, rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = pd1Controller(desiredLBState.ang, rs.lLeg.halfB.motorAngle, desiredLBState.vel, rs.lLeg.halfB.motorVelocity);
		co.rLeg.motorCurrentA = pd3Controller(desiredRAState.ang, rs.rLeg.halfA.motorAngle, desiredRAState.vel, rs.rLeg.halfA.motorVelocity);
		co.rLeg.motorCurrentB = pd4Controller(desiredRBState.ang, rs.rLeg.halfB.motorAngle, desiredRBState.vel, rs.rLeg.halfB.motorVelocity);

		// Change state if startup is no longer occurring.
		if (!isStarting()) {
			//printf("Startup is finished\n");
			state = 2;
			sw_flight = true;
			sw_stance = true;
			t=1;
		}
		return;
	}

	// calculate leg angle and length

	amp=guiIn.l_leg_st-guiIn.l_leg_fl;
	phi_rLeg = (rs.rLeg.halfA.motorAngle+rs.rLeg.halfB.motorAngle)/2;		        //right leg angle
	phi_lLeg = (rs.lLeg.halfA.motorAngle+rs.lLeg.halfB.motorAngle)/2;		        //left leg angle


	// calculate circle correction for step length
	c_radius_i = abs((1 - guiIn.lhip_pos / 2.3 ) / 2);
	c_radius_o = abs((1 - guiIn.rhip_pos / 2.3 ) / 2);
	gamma = guiIn.pea - guiIn.aea;
	oPEA = guiIn.pea + c_radius_o * gamma;
	iPEA = guiIn.pea - c_radius_i * gamma;
	oAEA = guiIn.aea - c_radius_o * gamma;
	iAEA = guiIn.aea + c_radius_i * gamma;
	phiBs_des_i = iPEA + acos (guiIn.l_leg_st);							//desired motor position for stance MOTOR B
	phiBs_des_o = oPEA + acos (guiIn.l_leg_st);							//desired motor position for stance MOTOR B


	switch (guiIn.control)
	{
		case 1:
			if (state == 2 && t>0.97 && rGC)								//switch left / right stepping 
			{
				state = 1;
				sw_stance = false;
				sw_flight = false;
				t=0;
			}
			if (state == 1 && t>0.97 && lGC) 	
			{
				state = 2;
				sw_stance = false;
				sw_flight = false;
				t=0;
			}
			break;
		case 2:
			// state machine
			//if (state == 2 && phi_rLeg<M_PI/2 && rGC)								//switch left / right stepping automatically
			if ((state == 2) && (t>0.97))
			{
				state = 1;
				sw_stance = false;
				sw_flight = false;
				t=0;
			}
			//if (state == 1 && phi_lLeg < M_PI/2 && lGC) 	
			if ((state == 1) && (t>0.97))
			{
				state = 2;
				sw_stance = false;
				sw_flight = false;
				t=0;
			}
			break;
		default:
			break;
	}

	//printf("t: %f \n",t);
	// command legs ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//************************************************************stance control right leg***************************************************************************************************************************
	        switch (state)                                                                                                                                                                                                                                                  //stance leg right, flight leg left
        {
                case 1:
                        s = 1 - (oPEA - phi_rLeg) / (oPEA - oAEA);
                        if (s > t)
                        {       
							t=s;
                        }
                        pd3Controller.D = guiIn.d_ls;
                        pd3Controller.P = guiIn.p_ls;
                        pd4Controller.D = guiIn.d_ls;
                        pd4Controller.P = guiIn.p_ls;
                        if ((rs.rLeg.halfB.motorAngle < phiBs_des_o) && !sw_stance)
                        {                                                                                                                                                                                               // stance leg rotate to pea
                                // asymmetry - extend right leg
                                std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(phi_rLeg,guiIn.l_leg_st);
                                co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0.5,rs.rLeg.halfB.motorVelocity) + guiIn.l_st;
                                co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0.5,rs.rLeg.halfA.motorVelocity);
                        } else
                        {                                                                                                                                                                                                                                                        // if pea was reached once
                                sw_stance=true;
                                std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(guiIn.pea, guiIn.l_leg_st);
                                co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity);
                                co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
                        }

                        //****************************************************************flight control left leg*************************************************************************************************************
                        if ((t < 1) && (!sw_flight))
                        {
                                pd0Controller.P = guiIn.p_lf;
                                pd0Controller.D = guiIn.d_lf;
                                pd1Controller.P = guiIn.p_lf;
                                pd1Controller.D = guiIn.d_lf;
                                if (t < 0.1){
                                        l_swing = guiIn.l_leg_st - amp * sin(t / guiIn.l_fl * M_PI);
                                        phi_lLeg = iPEA;
                                        logOut.state=11;
                                } else if (t<guiIn.l_fl){
                                        phi_lLeg=iPEA - (t - 0.1) / (guiIn.l_fl - 0.1) * (iPEA - iAEA) * (1 + guiIn.d_as);
                                        l_swing = guiIn.l_leg_st - amp * sin (t / guiIn.l_fl * M_PI);
                                        logOut.state=12;
                                } else {
                                        phi_lLeg=guiIn.aea - (1 - t) / (1 - guiIn.l_fl) * (iPEA-iAEA) * guiIn.d_as;
                                        l_swing = guiIn.l_leg_st;
                                        logOut.state=13;
                                }
                                //map leg angle sweep of flight leg to 0->1
                                std::tie(leftMotorAngle.A, leftMotorAngle.B) = commonToolkit.legPos2MotorPos(phi_lLeg,l_swing);
                                co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
                                co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity);
								if ((t < 0.1) & (leftMotorAngle.B < (rs.lLeg.halfB.legAngle - (rs.lLeg.halfA.motorAngle - rs.lLeg.halfB.legAngle))))
								{
									co.lLeg.motorCurrentB = guiIn.d_af;
                                    logOut.state=11.1;
								}

                        } else
                        {                                                                                                                                                                                                                                               // aea is reached once
                                sw_flight=true;
                                std::tie(leftMotorAngle.A, leftMotorAngle.B) = commonToolkit.legPos2MotorPos(iAEA,guiIn.l_leg_st);
                                pd0Controller.D = guiIn.d_ls;
                                pd0Controller.P = guiIn.p_ls;
                                co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
                                pd1Controller.D = guiIn.d_ls;
                                pd1Controller.P = guiIn.p_ls;
                                co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity)+2;
                                logOut.state=14;
                        }
						co.lLeg.motorCurrentA = clamp(co.lLeg.motorCurrentA,-25,25);
						co.lLeg.motorCurrentB = clamp(co.lLeg.motorCurrentB,-25,25);
                        break;


                        //********************************************************************************************************************************************************************************
                case 2:                         // stance leg left, swing leg right
                        s = 1 - (iPEA - phi_lLeg) / (iPEA - iAEA);
                        if (s > t)
                        {       
							t=s;
                        }
                        pd0Controller.D = guiIn.d_ls;
                        pd0Controller.P = guiIn.p_ls;
                        pd1Controller.D = guiIn.d_ls;
                        pd1Controller.P = guiIn.p_ls;
                        if ((rs.lLeg.halfB.motorAngle < phiBs_des_i) && !sw_stance)
                        {           // stance leg rotate to pea
                                std::tie(leftMotorAngle.A, leftMotorAngle.B) = commonToolkit.legPos2MotorPos(phi_lLeg,guiIn.l_leg_st);
                                co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0.5,rs.lLeg.halfB.motorVelocity)  + guiIn.l_st;
                                co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0.5,rs.lLeg.halfA.motorVelocity);
                        } else
                        {                        // if aea was reached once
                                sw_stance=true;
                                std::tie(leftMotorAngle.A, leftMotorAngle.B) = commonToolkit.legPos2MotorPos(guiIn.pea,guiIn.l_leg_st);
                                co.lLeg.motorCurrentB = pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity);
                                co.lLeg.motorCurrentA = pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
                                //logOut.currState = 4;
                        }

                        //*******************************************************************************************************************************************************************************************************************
                        if ((t < 1) && (!sw_flight))
                        {
                                pd3Controller.P = guiIn.p_lf;
                                pd3Controller.D = guiIn.d_lf;
                                pd4Controller.P = guiIn.p_lf;
                                pd4Controller.D = guiIn.d_lf;
                                if (t < 0.1){
                                        l_swing = guiIn.l_leg_st - amp * sin(t / guiIn.l_fl * M_PI);
                                        phi_rLeg = oPEA;
                                        logOut.state=21;
                                } else if (t<guiIn.l_fl){
                                        phi_rLeg=oPEA - (t - 0.1) / (guiIn.l_fl - 0.1) * (oPEA - oAEA) * (1 + guiIn.d_as);
                                        l_swing = guiIn.l_leg_st - amp * sin (t / guiIn.l_fl * M_PI);
                                        logOut.state=22;
                                } else {
                                        phi_rLeg=oAEA - (1 - t) / (1 - guiIn.l_fl) * (oPEA-oAEA) * guiIn.d_as;
                                        l_swing = guiIn.l_leg_st;
                                        logOut.state=23;
                                }
                                //map leg angle sweep of flight leg to 0->1
                                std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(phi_rLeg,l_swing);
                                co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
                                co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity);
								if ((t < 0.1) & (rightMotorAngle.B < (rs.rLeg.halfB.legAngle - (rs.rLeg.halfA.motorAngle - rs.rLeg.halfB.legAngle))))
								{
									co.rLeg.motorCurrentB=guiIn.d_af;
                                    logOut.state=21.1;
                                }

                        } else
                        {                                                                                                                                                                                                                                               // aea is reached once
                                sw_flight=true;
                                std::tie(rightMotorAngle.A, rightMotorAngle.B) = commonToolkit.legPos2MotorPos(oAEA,guiIn.l_leg_st);
                                pd3Controller.D = guiIn.d_ls;
                                pd3Controller.P = guiIn.p_ls;
                                co.rLeg.motorCurrentA = pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
                                pd4Controller.D = guiIn.d_ls;
                                pd4Controller.P = guiIn.p_ls;
                                co.rLeg.motorCurrentB = pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity)+2;
                                logOut.state=24;
                        }
						co.rLeg.motorCurrentA = clamp(co.rLeg.motorCurrentA,-25,25);
							   co.rLeg.motorCurrentB = clamp(co.rLeg.motorCurrentB,-25,25);
						  break;


                default:
                        break;
        } // end switch


	// Stuff the msg
	//co.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
	//co.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
	//co.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

	// end control code //
}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCEqPoint)

}
}

// vim: noexpandtab
