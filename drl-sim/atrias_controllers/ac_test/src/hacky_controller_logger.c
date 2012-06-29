#define BUFFER_SIZE 200000000 // 200 MB
#define STOP_POINT  190000000 // Where to stop the logging at

#include <fcntl.h>

char* log_data;
int cur_location;

void reset_log();

void init_logger() {
	log_data = (char *) malloc(BUFFER_SIZE);
	reset_log();
}

void reset_log() {
	memset(log_data, 0, BUFFER_SIZE);
	cur_location = 0;
}

void log_write(robot_state state, InputData* id, ControllerOutput* output, bool in_flight, float des_hip_angle, float stance_trigger_height, float des_leg_length,
	float des_mtr_angA, float des_mtr_angB, int stance_time) {
	if (cur_location > STOP_POINT) return;
	cur_location += snprintf((log_data + cur_location), BUFFER_SIZE - cur_location,
		"%u,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u,%u,%u,%u,%u,%u,%u,%u,%f,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u,%f,%f,%f,%f,%f,%u\n",
		state.command,
		state.loopTime,
		state.body_angle,
		state.body_angle_vel,
		state.body_pitch,
		state.body_pitch_vel,
		state.leg_angleA,
		state.leg_velocityA,
		state.motor_angleA,
		state.motor_angleA_inc,
		state.motor_torqueA,
		state.motor_velocityA,
		state.motorVoltageA,
		state.motor_currentA,
		state.desired_motor_angleA,
		state.desired_spring_defA,
		state.logicVoltageA,
		state.leg_angleB,
		state.leg_velocityB,
		state.motor_angleB,
		state.motor_angleB_inc,
		state.motor_torqueB,
		state.motor_velocityB,
		state.motorVoltageB,
		state.motor_currentB,
		state.desired_motor_angleB,
		state.desired_spring_defB,
		state.logicVoltageB,
		state.motor_angle_hip,
		state.motor_torque_hip,
		state.motor_velocity_hip,
		state.xPosition,
		state.yPosition,
		state.zPosition,
		state.xVelocity,
		state.yVelocity,
		state.zVelocity,
		state.horizontal_velocity,
		state.thermistorA[0],
		state.thermistorA[1],
		state.thermistorA[2],
		state.thermistorB[0],
		state.thermistorB[1],
		state.thermistorB[2],
		state.medullaStatusA,
		state.medullaStatusB,
		state.medullaStatusHip,
		state.medullaStatusBoom,
		state.limitSwitchA,
		state.limitSwitchB,
		state.limitSwitchHip,
		state.toe_switch,
		state.phase,
		state.in_flight ? 1 : 0,
		state.des_leg_angle,
		state.des_leg_length,
		state.des_hip_angle,
		state.flightHipP,
		state.flightHipD,
		state.flightKneeP,
		state.flightKneeD,
		state.stanceHipP,
		state.stanceHipD,
		state.stanceKneeP,
		state.stanceKneeD,
		state.stance_time,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	(float) 0.0,
    	output->motor_torqueA,
    	output->motor_torqueB,
    	output->motor_torque_hip,
    	output->in_flight,
    	output->des_leg_angle,
    	output->des_leg_length,
    	output->des_hip_angle,
    	output->flightHipP,
    	output->flightHipD,
    	output->flightKneeP,
    	output->flightKneeD,
    	output->stanceHipP,
    	output->stanceHipD,
    	output->stanceKneeP,
    	output->stanceKneeD,
    	output->stance_time,
    	in_flight ? 1 : 0,
    	des_hip_angle,
    	stance_trigger_height,
    	des_leg_length,
    	des_mtr_angA,
    	des_mtr_angB,
    	stance_time);
}

void send_log() {
	int fd = creat("/tmp/hacky_controller_log.log", 755);
	if (fd < 0) printf("Open file FAILED!!!\n");
	if (write(fd, log_data, cur_location) <= 0)
		printf("Write FAILED!!!\n");
	close(fd);
}
