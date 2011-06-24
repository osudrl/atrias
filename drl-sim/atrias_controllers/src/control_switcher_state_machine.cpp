// Devin Koepl

#include <atrias_controllers/control_switcher_state_machine.h>

extern void control_switcher_state_machine(ControllerInput *controller_input, ControllerOutput *controller_output, 
	ControllerState *controller_state, ControllerData *controller_data)
{
	switch (controller_state->state)
	{
		case CSSM_STATE_DISABLED:
			if (controller_data->command == CMD_RUN)
			{
				controller_state->state = CSSM_STATE_ENABLED;
				PRINT_MSG( "Controller enabled.\n" );
			}
			else if ((controller_data->command == CMD_DISABLE) 
			&& (controller_data->controller_requested != controller_state->controller_loaded))
			{
				// The user requests a different controller.
				takedown_controller(controller_input, controller_output, controller_state, controller_data);
				switch_controllers(controller_state, controller_data);
				initialize_controller(controller_input, controller_output, controller_state, controller_data);
				PRINT_MSG( "Switching to controller %u.\n", controller_data->controller_requested );
			}

			break;
		case CSSM_STATE_ERROR:
			if (controller_data->command == CMD_DISABLE)
			{
				controller_state->state = CSSM_STATE_DISABLED;
				PRINT_MSG("Error -> Disabled.\n");
			}

			break;
		case CSSM_STATE_ENABLED:
			if (controller_data->command == CMD_DISABLE)
			{
				controller_state->state = CSSM_STATE_DISABLED;
				PRINT_MSG("Controller disabled.\n");
			}

			update_controller(controller_input, controller_output, controller_state, controller_data);

			// Return to the wrapper now.
			return;
		case CSSM_STATE_INIT:
			initialize_controller = &initialize_no_controller;
			update_controller = &update_no_controller;
			takedown_controller = &takedown_no_controller;

			initialize_controller(controller_input, controller_output, controller_state, controller_data);

			controller_state->state = CSSM_STATE_DISABLED;

			PRINT_MSG("Initializing controller.\n");

		break;
		case CSSM_STATE_FINI:
			takedown_controller(controller_input, controller_output, controller_state, controller_data);

			controller_state->state = CSSM_STATE_DISABLED;

			PRINT_MSG("Taking down controller.\n");

		break;
	}

	controller_output->motor_torqueA = controller_output->motor_torqueB = 0.;
}

extern void switch_controllers(ControllerState * controller_state, ControllerData * controller_data)
{
  switch (controller_data->controller_requested)
	{
		case NO_CONTROLLER:
		  initialize_controller = &initialize_no_controller;
		  update_controller = &update_no_controller;
		  takedown_controller = &takedown_no_controller;

		  break;
		case MOTOR_TORQUE_CONTROLLER:
		  initialize_controller = &initialize_motor_torque_controller;
		  update_controller = &update_motor_torque_controller;
		  takedown_controller = &takedown_motor_torque_controller;

		  break;
		case MOTOR_POSITION_CONTROLLER:
		  initialize_controller = &initialize_motor_position_controller;
		  update_controller = &update_motor_position_controller;
		  takedown_controller = &takedown_motor_position_controller;

		  break;
		case LEG_TORQUE_CONTROLLER:
		  initialize_controller = &initialize_leg_torque_controller;
		  update_controller = &update_leg_torque_controller;
		  takedown_controller = &takedown_leg_torque_controller;

		  break;
		case LEG_POSITION_CONTROLLER:
		  initialize_controller = &initialize_leg_position_controller;
		  update_controller = &update_leg_position_controller;
		  takedown_controller = &takedown_leg_position_controller;

		  break;
		case SINE_WAVE_CONTROLLER:
		  initialize_controller = &initialize_leg_angle_sin_wave;
		  update_controller = &update_leg_angle_sin_wave;
		  takedown_controller = &takedown_leg_angle_sin_wave;

		  break;
		case RAIBERT_CONTROLLER:
		  initialize_controller = &initialize_raibert_controller;
		  update_controller = &update_raibert_controller;
		  takedown_controller = &takedown_raibert_controller;

		  break;
		case EQU_GAIT_CONTROLLER:
		  initialize_controller = &initialize_no_controller;
		  update_controller = &update_no_controller;
		  takedown_controller = &takedown_no_controller;

		  break;
		case TEST_CONTROLLER:
		  initialize_controller = &initialize_test_controller;
		  update_controller = &update_test_controller;
		  takedown_controller = &takedown_test_controller;

		  break;
	}

	controller_state->controller_loaded = controller_data->controller_requested;
}
