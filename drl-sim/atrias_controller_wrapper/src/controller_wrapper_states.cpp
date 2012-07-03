/*
 * controller_wrapper_states.cpp
 *
 *  Created on: Jun 12, 2012
 *      Author: Michael Anderson
 *
 *  The code in this file facilitates the transfer of information between the
 *  medulla communication structs and the controller wrapper.
 */

#include <atrias_controller_wrapper/controller_wrapper_states.h>

/**
 * @brief Switch between states and call the appropriate functions.
 *
 * @param uc_in array of pointers to each medualla input struct.
 * @param uc_out array of pointers to each medualla output struct.
*/
void control_wrapper_state_machine( uControllerInput ** uc_in, uControllerOutput ** uc_out ) {
    int i;

    if (uc_out[A_INDEX]) {
        buffer[0][(bufferPos[0]++)] = uc_out[A_INDEX]->TRANS_ANGLE;
        buffer[1][(bufferPos[1]++)] = uc_out[A_INDEX]->LEG_SEG_ANGLE;
    }
    if (uc_out[A_INDEX]) {
        buffer[2][(bufferPos[2]++)] = uc_out[B_INDEX]->TRANS_ANGLE;
        buffer[3][(bufferPos[3]++)] = uc_out[B_INDEX]->LEG_SEG_ANGLE;
    }

    bufferPos[0] %= 128;
    bufferPos[1] %= 128;
    bufferPos[2] %= 128;
    bufferPos[3] %= 128;
    encAverage[0] = 0;
    encAverage[1] = 0;
    encAverage[2] = 0;
    encAverage[3] = 0;

    for (i = 0; i < 128; i++) {
        encAverage[0] += buffer[0][i];
        encAverage[1] += buffer[1][i];
        encAverage[2] += buffer[2][i];
        encAverage[3] += buffer[3][i];
    }

    encAverage[0] /= 128;
    encAverage[1] /= 128;
    encAverage[2] /= 128;
    encAverage[3] /= 128;

    static unsigned char last_state = STATE_IDLE;
    static unsigned char next_state = STATE_IDLE;

    switch ( next_state )
    {
        case STATE_IDLE:
            next_state = state_wakeup( uc_in, uc_out );
            last_state = STATE_IDLE;
            break;
        case STATE_INIT:
            next_state = state_initialize(uc_in, uc_out, last_state);
            last_state = STATE_INIT;
            break;
        case STATE_RUN:
            next_state = state_run( uc_in, uc_out, last_state );
            last_state = STATE_RUN;
            break;
        case STATE_ERROR:
            next_state = state_error( uc_in, uc_out, last_state );
            last_state = STATE_ERROR;
            break;
        default:
            next_state = STATE_ERROR;
            break;
    }

    if ( cwd.controllerInput.command == CMD_RESTART )
        next_state = STATE_IDLE;
}

/*****************************************************************************/

/**
 * @breif wake up all the medullas on the robot.
 *
 * @param uc_in array of pointers to each medualla input struct.
 * @param uc_out array of pointers to each medualla output struct.
 *
 * @retrun  result to the response of a bad command (STATE_RESTART or STATE_WAKEUP).
 */
unsigned char state_wakeup( uControllerInput ** uc_in, uControllerOutput ** uc_out )
{
    int i;

    if (uc_in[A_INDEX]) {
        uc_in[A_INDEX]->enc_max = MEDULLA_A_ENC_MAX;
        uc_in[A_INDEX]->enc_min = MEDULLA_A_ENC_MIN;
    }
    if (uc_in[B_INDEX]) {
        uc_in[B_INDEX]->enc_max = MEDULLA_B_ENC_MAX;
        uc_in[B_INDEX]->enc_min = MEDULLA_B_ENC_MIN;
    }

    // Check to see if Medullas are awake by sending them a bad command (0).
    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
    {
        if (uc_in[i])
            uc_in[i]->command = STATE_INIT;
    }

    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ ) {
        // If a Medulla does not report the bad command, then fail.
        if (uc_out[i]) {
            if ( uc_out[i]->state != STATE_INIT) {
                //printf( "Medulla %u is in state %u instead of STATE_INIT (%u).\n", i, uc_out[i]->state, STATE_INIT );
                return STATE_IDLE;
            }
        }
    }

    // If successful, Medullas are ready to receive restarts.
    return STATE_INIT;
}

/*****************************************************************************/

// Initialize the encoder counters.
unsigned char state_initialize( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
    int i;
    // Check to see if Medullas are awake by sending them a bad command (0).
    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
    {
        if (uc_in[i]) {
            uc_in[i]->command = STATE_RUN;
            last_medulla_time[i] = uc_out[i]->timestep;
        }
    }

    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ ) {
        if (uc_out[i]) {
            // If a Medulla does not report the bad command, then fail.
            if ( uc_out[i]->state != STATE_RUN)
            {
                //printf( "Medulla %u is not in STATE_RUN\n", i );
                return STATE_INIT;
            }
        }
    }

    // If successful, Medullas are ready to receive restarts.
    if(uc_out[A_INDEX]) {
        motorAinc_offset_ticks = (int) uc_out[A_INDEX]->encoder[2];
        motorAinc_offset_angle = LEG_A_TRAN_ENC_TO_ANGLE(uc_out[A_INDEX]->TRANS_ANGLE) - TRAN_A_OFF_ANGLE;
        legAoffset = ((float)LEG_A_TRAN_ENC_TO_ANGLE(encAverage[0])) - ((float)LEG_A_LEG_ENC_TO_ANGLE(encAverage[1]));
    }
    if(uc_out[B_INDEX]) {
        motorBinc_offset_ticks = (int) uc_out[B_INDEX]->encoder[2];
        motorBinc_offset_angle = LEG_B_TRAN_ENC_TO_ANGLE(uc_out[B_INDEX]->TRANS_ANGLE) - TRAN_B_OFF_ANGLE;
        legBoffset = ((float)LEG_B_TRAN_ENC_TO_ANGLE(encAverage[2])) - ((float)LEG_B_LEG_ENC_TO_ANGLE(encAverage[3]));
    }
    if(uc_out[HIP_INDEX])
        hip_base_count = uc_out[HIP_INDEX]->encoder[1];

    if (uc_out[B_INDEX] && uc_out[A_INDEX]) {
        if (uc_out[B_INDEX]->state == STATE_RUN && uc_out[A_INDEX]->state == STATE_RUN)
            averageCounter++;
    }
    else {
        return STATE_RUN;
    }

    if (averageCounter < 128)
        return STATE_INIT;
    else
        return STATE_RUN;
}

/*****************************************************************************/

/*! Calculate velocity if dt is not zero.
 *  \param velocity  The velocity to be updated.
 *  \param curAngle  Current angle.
 *  \param prevAngle Previous angle.
 *  \param curTime   Current time.
 *  \param prevTime  Previous time.
 */
int calculate_velocity(float& velocity, float& curVal, float& prevVal, uint32_t& curTime, uint32_t& prevTime) {
    if (curTime != prevTime) {
        velocity = (curVal - prevVal) / ((float) (curTime - prevTime) * SEC_PER_CNT1);
        prevVal = curVal;
        return 0;
    }
    else {
        return 1;
    }
}

unsigned char state_run( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
    int i = 0;
    for (i = 0; i < 4; i++) {
        if (uc_out[i]) {
            if ((uc_out[i]->timestep - last_medulla_time[i]) > 100000) {
                current_medulla_time[i] =  ((uint32_t) 0xFFFF)+((uint32_t)uc_out[i]->timestep);
            }
            else {
                current_medulla_time[i] = uc_out[i]->timestep;
            }
        }
    }

    // Medulla stuff for motor A
    if (uc_out[A_INDEX]) {
        // Generate controller input
        cwd.controllerStatus.state.leg_angleA       = LEG_A_LEG_ENC_TO_ANGLE(uc_out[A_INDEX]->LEG_SEG_ANGLE);
        cwd.controllerStatus.state.motor_angleA     = LEG_A_TRAN_ENC_TO_ANGLE(uc_out[A_INDEX]->TRANS_ANGLE) - legAoffset;// - TRAN_A_OFF_ANGLE;
        cwd.controllerStatus.state.motor_angleA_inc = motorAinc_offset_angle + ((((int)uc_out[A_INDEX]->encoder[2])-motorAinc_offset_ticks)*INC_ENCODER_RAD_PER_TICK);

        // Calculate velocities.
        medulla_bad_timestamp_counter[A_INDEX] += calculate_velocity(cwd.controllerStatus.state.motor_velocityA, cwd.controllerStatus.state.motor_angleA, last_motor_angleA, current_medulla_time[A_INDEX], last_medulla_time[A_INDEX]);
        medulla_bad_timestamp_counter[A_INDEX] += calculate_velocity(cwd.controllerStatus.state.leg_velocityA, cwd.controllerStatus.state.leg_angleA, last_leg_angleA, current_medulla_time[A_INDEX], last_medulla_time[A_INDEX]);

        // Update status variabes
        cwd.controllerStatus.state.thermistorA[0] =ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[0]);
        cwd.controllerStatus.state.thermistorA[1] =ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[1]);
        cwd.controllerStatus.state.thermistorA[2] =ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[2]);
	cwd.controllerStatus.state.limit_switch_A = uc_out[A_INDEX]->limitSW;

        cwd.controllerStatus.state.motorVoltageA =POWER_ADC_TO_V(uc_out[A_INDEX]->motor_power);
        cwd.controllerStatus.state.logicVoltageA =POWER_ADC_TO_V(uc_out[A_INDEX]->logic_power);
        cwd.controllerStatus.state.medullaStatusA |= uc_out[A_INDEX]->error_flags;

        uc_in[A_INDEX]->counter++;
    }


    // Medulla stuff for motor B
    if (uc_out[B_INDEX]) {
        // Generate controller input
        cwd.controllerStatus.state.leg_angleB       = LEG_B_LEG_ENC_TO_ANGLE(uc_out[B_INDEX]->LEG_SEG_ANGLE);
        cwd.controllerStatus.state.motor_angleB     = LEG_B_TRAN_ENC_TO_ANGLE(uc_out[B_INDEX]->TRANS_ANGLE) - legBoffset;// - TRAN_B_OFF_ANGLE;
        cwd.controllerStatus.state.motor_angleB_inc = motorBinc_offset_angle + ((((int)uc_out[B_INDEX]->encoder[2])-motorBinc_offset_ticks)*INC_ENCODER_RAD_PER_TICK);

        // Calculate velocities.
        medulla_bad_timestamp_counter[B_INDEX] += calculate_velocity(cwd.controllerStatus.state.motor_velocityB, cwd.controllerStatus.state.motor_angleB, last_motor_angleB, current_medulla_time[B_INDEX], last_medulla_time[B_INDEX]);
        medulla_bad_timestamp_counter[B_INDEX] += calculate_velocity(cwd.controllerStatus.state.leg_velocityB, cwd.controllerStatus.state.leg_angleB, last_leg_angleB, current_medulla_time[B_INDEX], last_medulla_time[B_INDEX]);

        // Update status variabes
        cwd.controllerStatus.state.thermistorB[0] =ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[0]);
        cwd.controllerStatus.state.thermistorB[1] =ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[1]);
        cwd.controllerStatus.state.thermistorB[2] =ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[2]);
	cwd.controllerStatus.state.limit_switch_B = uc_out[B_INDEX]->limitSW;

        cwd.controllerStatus.state.motorVoltageB =POWER_ADC_TO_V(uc_out[B_INDEX]->motor_power);
        cwd.controllerStatus.state.logicVoltageB =POWER_ADC_TO_V(uc_out[B_INDEX]->logic_power);
        cwd.controllerStatus.state.medullaStatusB |= uc_out[B_INDEX]->error_flags;

        cwd.controllerStatus.state.toe_switch = uc_out[B_INDEX]->toe_switch;

        uc_in[B_INDEX]->counter++;
    }

    // Medulla stuff for hip
    if (uc_out[HIP_INDEX]) {
        cwd.controllerStatus.state.motor_angle_hip = UNDISCRETIZE(uc_out[HIP_INDEX]->encoder[1], MAX_HIP_ANGLE, MIN_HIP_ANGLE, hip_base_count, hip_base_count+HIP_COUNT_RANGE);
        medulla_bad_timestamp_counter[HIP_INDEX] += calculate_velocity(cwd.controllerStatus.state.motor_velocity_hip, cwd.controllerStatus.state.motor_angle_hip, last_hip_angle, current_medulla_time[HIP_INDEX], last_medulla_time[HIP_INDEX]);

        cwd.controllerStatus.state.medullaStatusHip |= uc_out[HIP_INDEX]->error_flags;

        uc_in[HIP_INDEX]->counter++;
    }

    cwd.controllerStatus.state.command = cwd.controllerInput.command;

    // Medulla stuff for boom
    if (uc_out[BOOM_INDEX]) {
        // Update boom angles
        if (ABS((int32_t)(uc_out[BOOM_INDEX]->encoder[0]) - last_body_cnt) > 1000) {
        if (((int32_t)(uc_out[BOOM_INDEX]->encoder[0]) - last_body_cnt) < 0)
            body_angle_cnt_base += 0x1FFFF;
        else
            body_angle_cnt_base -= 0x1FFFF;
        }
        last_body_cnt = (int32_t)(uc_out[BOOM_INDEX]->encoder[0]);

        cwd.controllerStatus.state.medullaStatusBoom |= uc_out[BOOM_INDEX]->error_flags;

        uc_in[BOOM_INDEX]->counter++;

        cwd.controllerStatus.state.body_angle = (((float)((int32_t)(uc_out[BOOM_INDEX]->encoder[0])+body_angle_cnt_base))-BOOM_TILT_OFFSET)*BOOM_RAD_PER_CNT*BOOM_TILT_GEAR_RATIO;
        medulla_bad_timestamp_counter[BOOM_INDEX] += calculate_velocity(cwd.controllerStatus.state.body_angle_vel, cwd.controllerStatus.state.body_angle, last_body_angle, current_medulla_time[BOOM_INDEX], last_medulla_time[BOOM_INDEX]);

        cwd.controllerStatus.state.body_pitch = (((float)(uc_out[BOOM_INDEX]->encoder[2]))-BOOM_PITCH_OFFSET)*BOOM_PITCH_RAD_PER_CNT*BOOM_PITCH_GEAR_RATIO;
        medulla_bad_timestamp_counter[BOOM_INDEX] += calculate_velocity(cwd.controllerStatus.state.body_pitch_vel, cwd.controllerStatus.state.body_pitch, last_body_pitch, current_medulla_time[BOOM_INDEX], last_medulla_time[BOOM_INDEX]);

        cwd.controllerStatus.state.zPosition = BOOM_PIVOT_HEIGHT + BOOM_LENGTH*sin(cwd.controllerStatus.state.body_angle) + BOOM_ROBOT_OFFSET;
        medulla_bad_timestamp_counter[BOOM_INDEX] += calculate_velocity(cwd.controllerStatus.state.zVelocity, cwd.controllerStatus.state.zPosition, last_zPosition, current_medulla_time[BOOM_INDEX], last_medulla_time[BOOM_INDEX]);

        if (ABS((int32_t)(uc_out[BOOM_INDEX]->encoder[1]) - last_xPositionEncoder) > 10000) {   // Has the encoder wrapped around?
        if (((int32_t)(uc_out[BOOM_INDEX]->encoder[1]) - last_xPositionEncoder) > 0)
            xPositionBase -= ((2*PI)*BOOM_HOPPING_RADIUS*BOOM_PAN_GEAR_RATIO);
        else
            xPositionBase += ((2*PI)*BOOM_HOPPING_RADIUS*BOOM_PAN_GEAR_RATIO);
       }

        last_xPositionEncoder = (int32_t)(uc_out[BOOM_INDEX]->encoder[1]);
        cwd.controllerStatus.state.xPosition = (((float)(uc_out[BOOM_INDEX]->encoder[1]))*BOOM_RAD_PER_CNT*BOOM_HOPPING_RADIUS*BOOM_PAN_GEAR_RATIO) + xPositionBase;
        medulla_bad_timestamp_counter[BOOM_INDEX] += calculate_velocity(cwd.controllerStatus.state.xVelocity, cwd.controllerStatus.state.xPosition, last_xPosition, current_medulla_time[BOOM_INDEX], last_medulla_time[BOOM_INDEX]);
    }

    // Update the controller
    control_switcher_state_machine(cwd.controllerInput, cwd.controllerStatus);
    //printf("TORQUE A: %i\n", (int)cwd.controllerStatus.state.motor_torqueA);

    // Clamp the motor torques.
    cwd.controllerStatus.state.motor_torqueA = CLAMP(cwd.controllerStatus.state.motor_torqueA, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);
    cwd.controllerStatus.state.motor_torqueB = CLAMP(cwd.controllerStatus.state.motor_torqueB, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);
    cwd.controllerStatus.state.motor_torque_hip = CLAMP(cwd.controllerStatus.state.motor_torque_hip, MTR_MIN_HIP_TRQ_LIMIT, MTR_MAX_HIP_TRQ_LIMIT);

    if (uc_in[A_INDEX])
        uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
                cwd.controllerStatus.state.motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    if (uc_in[B_INDEX])
        uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
                -cwd.controllerStatus.state.motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    if (uc_in[HIP_INDEX])
        uc_in[HIP_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            cwd.controllerStatus.state.motor_torque_hip, MTR_MIN_HIP_TRQ, MTR_MAX_HIP_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);


    for (i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++) {
        if (uc_out[i])
            last_medulla_time[i] = uc_out[i]->timestep;
    }

    for (i = 0; i > NUM_OF_MEDULLAS_ON_ROBOT; i++) {
        if (uc_out[i]) {
            if (uc_out[i]->state == 5)   // If the medulla is in error
                return STATE_ERROR;
        }
    }

    return STATE_RUN;
}

/*****************************************************************************/

unsigned char state_error (uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state) {
    // Send zero motor torques.
    if (uc_in[A_INDEX])
        uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    if (uc_in[B_INDEX])
        uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    if (uc_in[HIP_INDEX])
        uc_in[HIP_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);

    // No coming back from this one yet.
    return STATE_ERROR;
}

