/*
 * Acts as the proxy between the ethercat master and the controller libraries
 *
 * Author: Michael Anderson
 */

#include <atrias_controller_switcher/control_switcher_state_machine.h>

extern void control_switcher_state_machine(controller_input &input, controller_status &status)
{
    switch (state)
    {
        case CSSM_STATE_DISABLED:
            if (input.command == CMD_RUN)
            {
            	state = status.cssm_state = CSSM_STATE_ENABLED;
                //printf("Controller enabled.\n");
            }
            else if ((input.command == CMD_DISABLE)
                    && (input.controller_requested != currentController))
            {
                // The user requests a different controller.
                switch_controllers(input.controller_requested);
            }
        	status.state.motor_torqueA = status.state.motor_torqueB = status.state.motor_torque_hip = 0.;
            status.cssm_state = state;

            break;
        case CSSM_STATE_ERROR:
            if (input.command == CMD_DISABLE)
            {
                state = CSSM_STATE_DISABLED;
                //printf("Error -> Disabled.\n");
            }
            status.cssm_state = state;

            break;
        case CSSM_STATE_ENABLED:
            if (input.command == CMD_DISABLE || input.command == CMD_RESTART)
            {
            	state = status.cssm_state = CSSM_STATE_DISABLED;
                //printf("Controller disabled.\n");
            }
            if (controller_loaded) {
            	robot_state tmp = status.state;
            	ControllerOutput output;
				VECTOR_TO_BYTE_ARRAY(input.controller_input, cInput);
				output.motor_torqueA = output.motor_torqueB = output.motor_torque_hip = 0.;
				update_controller(tmp, cInput, &output, cStatus);
				byteArrayToVector(cStatus, status.status);

				//Clamp and set the motor torques
				status.state.motor_torqueA = CLAMP(output.motor_torqueA, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);
				status.state.motor_torqueB = CLAMP(output.motor_torqueB, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);
				status.state.motor_torque_hip = CLAMP(output.motor_torque_hip, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);

                status.state.in_flight        = output.in_flight;
                status.state.des_leg_angle    = output.des_leg_angle;
                status.state.des_leg_length   = output.des_leg_length;
                status.state.des_hip_angle    = output.des_hip_angle;
                status.state.motor_torqueA    = output.motor_torqueA;
                status.state.motor_torqueB    = output.motor_torqueB;
                status.state.motor_torque_hip = output.motor_torque_hip;
                status.state.flightHipP       = output.flightHipP;
                status.state.flightHipD       = output.flightHipD;
                status.state.flightKneeP      = output.flightKneeP;
                status.state.flightKneeD      = output.flightKneeD;
                status.state.stanceHipP       = output.stanceHipP;
                status.state.stanceHipD       = output.stanceHipD;
                status.state.stanceKneeP      = output.stanceKneeP;
                status.state.stanceKneeD      = output.stanceKneeD;
                status.state.stance_time      = output.stance_time;

				status.cssm_state = state;
    		}
            else {
            	state = status.cssm_state = CSSM_STATE_ERROR;
            }
            // Return to the wrapper now.
            return;
        case CSSM_STATE_INIT:
            controller_loaded = false;
            byteArraysInitialized = false;
            state = status.cssm_state = CSSM_STATE_DISABLED;

            break;
        case CSSM_STATE_FINI:
        	if (controller_loaded)
        		takedown_controller();

            state = status.cssm_state = CSSM_STATE_DISABLED;

            break;
        case CSSM_STATE_LOAD_FAIL:
            status.cssm_state = state;
        	state = CSSM_STATE_DISABLED;
        	break;
    }
}

extern void switch_controllers(std::string name)
{
	controller_loaded = false;

	if (byteArraysInitialized) {
		FREE_BYTE_ARRAY(cInput);
		FREE_BYTE_ARRAY(cStatus);
		byteArraysInitialized = false;
	}
	if (takedown_controller)
		takedown_controller();
	if (controller_handle)
		dlclose(controller_handle);

	if (name != "" && name != "none") {
		currentController = name;

		//TODO: Make sure getting ros package paths is realtime-safe (probably not)
		controller_metadata md = loadControllerMetadata(ros::package::getPath(name), name);
		controller_handle = dlopen(md.coreLibPath.c_str(), RTLD_NOW);

		if (!controller_handle) {
			printf("Controller library loading failed!\n");
			printf("Library path: %s\n", md.coreLibPath.c_str());
			state = CSSM_STATE_LOAD_FAIL;
			return;
		}

		initialize_controller = (ControllerInitResult(*)())dlsym(controller_handle, "controllerInit");
		update_controller = (void(*)(robot_state, ByteArray, ControllerOutput*, ByteArray&))dlsym(controller_handle, "controllerUpdate");
		takedown_controller = (void(*)())dlsym(controller_handle, "controllerTakedown");

		if (initialize_controller && update_controller && takedown_controller) {
			ControllerInitResult cir = initialize_controller();
			if (!cir.error) {
				cInput = NEW_BYTE_ARRAY(cir.controllerInputSize);
				cStatus = NEW_BYTE_ARRAY(cir.controllerStatusSize);
				byteArraysInitialized = true;
				controller_loaded = true;
				return;
			}
			printf("Controller initialization function returned an error!\n");
			state = CSSM_STATE_LOAD_FAIL;
			return;
		}
		if (!initialize_controller)
			printf("Controller library initialization function is missing!\n");
		if (!update_controller)
			printf("Controller library update function is missing!\n");
		if (!takedown_controller)
			printf("Controller library takedown function is missing!\n");
		state = CSSM_STATE_LOAD_FAIL;
	}
}
