/*
 * @file
 * @author Devin Koepl
 *
 * @breif atrias ethercat code. Makes a kernel modul.
 *
 *
 *
 */
#include "controller_wrapper_states.h"

/******************************************************************************/

#define STATE_IDLE          0
#define STATE_INIT          1
#define STATE_RUN           2
#define STATE_ERROR         3

/*****************************************************************************/

/**
 * @brief pointer to shared memory stuct for shared memory management
 */
static Shm * shm;

/*****************************************************************************/

// Control wrapper variables.

unsigned short int  tranA_off;
unsigned short int  tranB_off;

int                 boom_pan_off  = 0;
int                 boom_tilt_off = 0;

unsigned short int  last_boom_pan_cnt;
unsigned short int  first_boom_pan_cnt;
unsigned short int  last_boom_tilt_cnt;

float               boom_pan_angle  = 0.;
float               boom_tilt_angle = 0.;

float               last_boom_pan_angle = 0.;
float               last_body_angle = 0.;
int32_t	    	    body_angle_cnt_base = 0;
int32_t	    	    last_body_cnt = 0;
float		    last_body_pitch = 0.;

unsigned short int  last_tranA_cnt;
unsigned short int  last_tranB_cnt;

float               last_motor_angleA = 0.;
float               last_motor_angleB = 0.;
float               last_leg_angleA   = 0.;
float               last_leg_angleB   = 0.;
float               last_hip_angle    = 0.;

float		    legAoffset = 0.0;
float		    legBoffset = 0.0;
int		    averageCounter = 0;

float               hor_vel;
float               hor_vel_buffer[HOR_VEL_WINDOW];
int                 hor_vel_index     = 0;

float               xPositionBase = 0;
int32_t             last_xPositionEncoder = 0;

float               last_zPosition = 0;
float               last_xPosition = 0;

float               leg_angle;
float               leg_length;

int                 motorAinc_offset_ticks = 0;
int                 motorBinc_offset_ticks = 0;
float               motorAinc_offset_angle = 0.0;
float               motorBinc_offset_angle = 0.0;
uint32_t	    last_medulla_time[NUM_OF_MEDULLAS_ON_ROBOT];
uint32_t	    current_medulla_time[NUM_OF_MEDULLAS_ON_ROBOT];

static unsigned char command = CMD_RUN;

ControllerInput*    c_in;
ControllerOutput*   c_out;

uint32_t            buffer[4][128];
int                 bufferPos[4];
uint64_t            encAverage[4];

int		    firstRun = 0;

/*****************************************************************************/

/**
 * @brief Initialize shared memory struct.
 *  
 * Zeros out the memory block and sets default state.
 *
 * @return True upon success.
 */
unsigned char initialize_shm( void )
{
    // To Uspace SHM
    if ( !( shm = ( Shm * )rt_shm_alloc( nam2num( SHM_NAME ), sizeof( Shm ), USE_VMALLOC ) ) )
        return false;
    memset( shm, 0, sizeof( Shm ) );

    shm->controller_data[0].command = shm->controller_data[1].command = CMD_DISABLE;
    shm->controller_data[0].controller_requested = shm->controller_data[1].controller_requested = NO_CONTROLLER;

    return true;
}

/*****************************************************************************/

/**
 * @breif Free the shared memory.
 */
void takedown_shm( void )
{
    rt_shm_free( nam2num( SHM_NAME ) );
}

/*****************************************************************************/

/**
 * @brief Switch between states and call the appropriate functions.
 *
 * @param uc_in array of pointers to each medualla input struct.  
 * @param uc_out array of pointers to each medualla output struct.
*/
void control_wrapper_state_machine( uControllerInput ** uc_in, uControllerOutput ** uc_out )
{
    int i;
/*
    printk("ThermistorA[0]: %d, %d\n", (uc_out[A_INDEX]->thermistor[0]), (int)(ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[0])));
    printk("ThermistorA[1]: %d, %d\n", (uc_out[A_INDEX]->thermistor[1]), (int)(ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[1])));
    printk("ThermistorA[2]: %d, %d\n", (uc_out[A_INDEX]->thermistor[2]), (int)(ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[2])));
    
    printk("ThermistorB[0]: %d, %d\n", (uc_out[B_INDEX]->thermistor[0]), (int)(ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[0])));
    printk("ThermistorB[1]: %d, %d\n", (uc_out[B_INDEX]->thermistor[1]), (int)(ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[1])));
    printk("ThermistorB[2]: %d, %d\n", (uc_out[B_INDEX]->thermistor[2]), (int)(ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[2])));
	
    printk("Motor Voltage: %d\n", (int)(POWER_ADC_TO_V(uc_out[A_INDEX]->motor_power)));
    printk("Logic Voltage: %d\n", (int)(POWER_ADC_TO_V(uc_out[A_INDEX]->logic_power)));
*/

    buffer[0][(bufferPos[0]++)] = uc_out[A_INDEX]->TRANS_ANGLE;
    buffer[1][(bufferPos[1]++)] = uc_out[A_INDEX]->LEG_SEG_ANGLE;
    buffer[2][(bufferPos[2]++)] = uc_out[B_INDEX]->TRANS_ANGLE;
    buffer[3][(bufferPos[3]++)] = uc_out[B_INDEX]->LEG_SEG_ANGLE;
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

    //printk("Trans Encoder A: %d\n",(int32_t)uc_out[A_INDEX]->encoder[2]);
    //printk("Motor Encoder A: %d\n",encAverage[0]);
    //printk("Leg Encoder   A: %d\n",encAverage[1]);
    //printk("Spring Deflection A: %d\n\n", (encAverage[0]-TRAN_A_CALIB_VAL)-(encAverage[1]-LEG_A_CALIB_VAL));
    
    //printk("Trans Encoder B: %d\n",(int32_t)uc_out[B_INDEX]->encoder[2]);
    //printk("Motor Encoder B: %d\n",encAverage[2]);
    //printk("Leg Encoder   B: %d\n\n",encAverage[3]);
    //printk("Spring Deflection B: %d\n\n", (encAverage[2]-TRAN_B_CALIB_VAL)-(encAverage[3]-LEG_B_CALIB_VAL));
    //printk("ERROR Code[A]: %d\n",uc_out[A_INDEX]->error_flags);
    //printk("   LimitSW[A]: %d\n\n",uc_out[A_INDEX]->limitSW);
    //printk("ERROR Code[B]: %d\n",uc_out[B_INDEX]->error_flags);
    //printk("ERROR Code[HIP]: %d\n",uc_out[HIP_INDEX]->error_flags);
    //printk("ERROR Code[BOOM]: %d\n",uc_out[BOOM_INDEX]->error_flags);
    //printk("   LimitSW[B]: %d\n\n\n",uc_out[B_INDEX]->limitSW);
    //printk("  Yaw Encoder: %d\n", uc_out[BOOM_INDEX]->encoder[0]);
    //printk("Pitch Encoder: %d\n", uc_out[BOOM_INDEX]->encoder[2]);
    //printk("Current %d\n", uc_out[A_INDEX]->encoder[3]);
    //printk("SizeOf: %d\n", 8*sizeof(uControllerOutput));
    //printk("Timestep: %d\n", uc_out[2]->timestep);
    //printk("                                                                                                Toe Switch: %d\n",uc_out[B_INDEX]->toe_switch);
    //printk("												Hip Encoder: %d,%d\n",uc_out[HIP_INDEX]->encoder[1],(int)(UNDISCRETIZE(uc_out[HIP_INDEX]->encoder[1], MAX_HIP_ANGLE, MIN_HIP_ANGLE, MIN_HIP_COUNT, MAX_HIP_COUNT)*1000.0));

    ///printk("%d,%d,%d,%d\n",uc_out[0]->error_flags,uc_out[1]->error_flags,uc_out[2]->error_flags,uc_out[3]->error_flags);

    // Keep a copy of the states in memory.
    static unsigned char last_state = STATE_IDLE;
    static unsigned char next_state = STATE_IDLE;

    switch ( next_state )
    {
        case STATE_IDLE:
            next_state = state_wakeup( uc_in, uc_out, last_state );
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
    }

    if ( shm->controller_data[shm->control_index].command == CMD_RESTART )
        next_state = STATE_IDLE;

    // Handle controller data change requests.
    if ( shm->req_switch )
    {
        shm->control_index = 1 - shm->control_index;
        shm->req_switch    = false;
    }
}

/*****************************************************************************/

/**
 * @breif wake up all the medullas on the robot.
 *
 * @param uc_in array of pointers to each medualla input struct.
 * @param uc_out array of pointers to each medualla output struct.
 * @param last_state is not used in the function, TODO: remove this param
 *
 * @retrun  result to the response of a bad command (STATE_RESTART or STATE_WAKEUP).
 */
unsigned char state_wakeup( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
    int i;

    uc_in[A_INDEX]->enc_max = MEDULLA_A_ENC_MAX;
    uc_in[A_INDEX]->enc_min = MEDULLA_A_ENC_MIN;

    uc_in[B_INDEX]->enc_max = MEDULLA_B_ENC_MAX;
    uc_in[B_INDEX]->enc_min = MEDULLA_B_ENC_MIN;

    // Check to see if Medullas are awake by sending them a bad command (0).
    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
    {
        uc_in[i]->command = STATE_INIT;
    }

    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ ) {
        // If a Medulla does not report the bad command, then fail.
        if ( uc_out[i]->state != STATE_INIT)
        {
            rt_printk( "Medulla %u is not in STATE_INIT\n", i );
            return STATE_IDLE;
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
    shm->controller_state.state = 3;
    // Check to see if Medullas are awake by sending them a bad command (0).
    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
    {
        uc_in[i]->command = STATE_RUN;
	last_medulla_time[i] = uc_out[i]->timestep;
    }

    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ ) {
        // If a Medulla does not report the bad command, then fail.
        if ( uc_out[i]->state != STATE_RUN)
        {
            rt_printk( "Medulla %u is not in STATE_RUN\n", i );
            return STATE_INIT;
        }
    }

    // If successful, Medullas are ready to receive restarts.
    motorAinc_offset_ticks = (int) uc_out[A_INDEX]->encoder[2];
    motorBinc_offset_ticks = (int) uc_out[B_INDEX]->encoder[2];
    motorAinc_offset_angle = LEG_A_TRAN_ENC_TO_ANGLE(uc_out[A_INDEX]->TRANS_ANGLE) - TRAN_A_OFF_ANGLE;
    motorBinc_offset_angle = LEG_B_TRAN_ENC_TO_ANGLE(uc_out[B_INDEX]->TRANS_ANGLE) - TRAN_B_OFF_ANGLE;

    legAoffset = ((float)LEG_A_TRAN_ENC_TO_ANGLE(encAverage[0])) - ((float)LEG_A_LEG_ENC_TO_ANGLE(encAverage[1]));
    legBoffset = ((float)LEG_B_TRAN_ENC_TO_ANGLE(encAverage[2])) - ((float)LEG_B_LEG_ENC_TO_ANGLE(encAverage[3]));
    //printk("A: %d - %d = %d\n",(int)(LEG_A_TRAN_ENC_TO_ANGLE(uc_out[A_INDEX]->TRANS_ANGLE)*1000.0),(int)(LEG_A_LEG_ENC_TO_ANGLE(uc_out[A_INDEX]->LEG_SEG_ANGLE)*1000.0),(int)(legAoffset*1000.0));
    //printk("B: %d - %d = %d\n",(int)(LEG_A_TRAN_ENC_TO_ANGLE(uc_out[B_INDEX]->TRANS_ANGLE)*1000.0),(int)(LEG_B_LEG_ENC_TO_ANGLE(uc_out[A_INDEX]->LEG_SEG_ANGLE)*1000.0),(int)(legBoffset*1000.0));

    if (uc_out[A_INDEX]->state == STATE_RUN)
	averageCounter++;


    //last_xPositionEncoder = (int32_t)(uc_out[BOOM_INDEX]->encoder[1]);
    
    //last_body_cnt = (int32_t)(uc_out[BOOM_INDEX]->encoder[0]);

    if (averageCounter < 128)
	return STATE_INIT;
    else
    	return STATE_RUN;
}

/*****************************************************************************/

unsigned char state_run( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
    int i = 0;
    for (i = 0; i < 3; i++) {
	if ((uc_out[i]->timestep - last_medulla_time[i]) > 100000) {
	    current_medulla_time[i] =  0xFFFF+uc_out[i]->timestep;
	    //printk("Overflow: %d - %d = %d\n", current_medulla_time[i],last_medulla_time[i],current_medulla_time[i]-last_medulla_time[i]);
	}
	else {
	    current_medulla_time[i] = uc_out[i]->timestep;
	    //printk("%d - %d = %d\n",current_medulla_time[i],last_medulla_time[i],current_medulla_time[i]-last_medulla_time[i]);
	}
    }

    // Create pointers to controller I/O
    c_in  = &shm->controller_input[shm->io_index];
    c_out = &shm->controller_output[shm->io_index];

    // Generate controller input
    c_in->leg_angleA       = LEG_A_LEG_ENC_TO_ANGLE(uc_out[A_INDEX]->LEG_SEG_ANGLE);
    c_in->leg_angleB       = LEG_B_LEG_ENC_TO_ANGLE(uc_out[B_INDEX]->LEG_SEG_ANGLE);
    c_in->motor_angleA     = LEG_A_TRAN_ENC_TO_ANGLE(uc_out[A_INDEX]->TRANS_ANGLE) - legAoffset;// - TRAN_A_OFF_ANGLE;
    c_in->motor_angleB     = LEG_B_TRAN_ENC_TO_ANGLE(uc_out[B_INDEX]->TRANS_ANGLE) - legBoffset;// - TRAN_B_OFF_ANGLE;
    //printk("Leg Time: %d\n",current_medulla_time[A_INDEX]-last_medulla_time[A_INDEX]);
    c_in->motor_angleA_inc = motorAinc_offset_angle + ((((int)uc_out[A_INDEX]->encoder[2])-motorAinc_offset_ticks)*INC_ENCODER_RAD_PER_TICK);
    c_in->motor_angleB_inc = motorBinc_offset_angle + ((((int)uc_out[B_INDEX]->encoder[2])-motorBinc_offset_ticks)*INC_ENCODER_RAD_PER_TICK);

    c_in->motor_velocityA = (c_in->motor_angleA - last_motor_angleA)
        / ( (float)(current_medulla_time[A_INDEX]-last_medulla_time[A_INDEX]) * SEC_PER_CNT1 );
    c_in->motor_velocityB = (c_in->motor_angleB - last_motor_angleB)
        / ( (float)(current_medulla_time[B_INDEX]-last_medulla_time[B_INDEX]) * SEC_PER_CNT1 );

    c_in->leg_velocityA = (c_in->leg_angleA - last_leg_angleA)
        / ( (float)(current_medulla_time[A_INDEX]-last_medulla_time[A_INDEX]) * SEC_PER_CNT1 );
    c_in->leg_velocityB = (c_in->leg_angleB - last_leg_angleB)
        / ( (float)(current_medulla_time[B_INDEX]-last_medulla_time[B_INDEX]) * SEC_PER_CNT1 );

    // Hip Inputs
    c_in->hip_angle = UNDISCRETIZE(uc_out[HIP_INDEX]->encoder[1], MAX_HIP_ANGLE, MIN_HIP_ANGLE, MIN_HIP_COUNT, MAX_HIP_COUNT);
    //c_in->hip_angle = ((float)(uc_out[HIP_INDEX]->encoder[1]))*(2.0*PI)/8192.0;
    c_in->hip_angle_vel = (c_in->hip_angle - last_hip_angle) / ((float)(current_medulla_time[HIP_INDEX]-last_medulla_time[HIP_INDEX]) * SEC_PER_CNT1 );
    //printk("%d\n",(c_in->hip_angle - last_hip_angle));
    last_hip_angle = c_in->hip_angle;
    c_in->toe_switch = uc_out[B_INDEX]->toe_switch;
    c_in->toe_force = uc_out[A_INDEX]->toe_switch;

    last_motor_angleA = c_in->motor_angleA;

    last_motor_angleB = c_in->motor_angleB;
    last_leg_angleA        = c_in->leg_angleA;
    last_leg_angleB        = c_in->leg_angleB;

    c_in->command = shm->controller_data[shm->control_index].command;

    // Update status variabes
    c_in->thermistorA[0] =ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[0]);
    c_in->thermistorA[1] =ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[1]);
    c_in->thermistorA[2] =ADC_TO_TEMP(uc_out[A_INDEX]->thermistor[2]);
    c_in->thermistorB[0] =ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[0]);
    c_in->thermistorB[1] =ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[1]);
    c_in->thermistorB[2] =ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[2]);

    c_in->motorVoltageA =POWER_ADC_TO_V(uc_out[A_INDEX]->motor_power);
    c_in->motorVoltageB =POWER_ADC_TO_V(uc_out[B_INDEX]->motor_power);

    c_in->logicVoltageA =POWER_ADC_TO_V(uc_out[A_INDEX]->logic_power);
    c_in->logicVoltageB =POWER_ADC_TO_V(uc_out[B_INDEX]->logic_power);

    c_in->medullaStatusA |= uc_out[A_INDEX]->error_flags;
    c_in->medullaStatusB |= uc_out[B_INDEX]->error_flags;
    c_in->medullaStatusHip |= uc_out[HIP_INDEX]->error_flags;
    //c_in->medullaStatusBoom |= uc_out[BOOM_INDEX]->error_flags;

    /*
    // Update boom angles
    if (ABS((int32_t)(uc_out[BOOM_INDEX]->encoder[0]) - last_body_cnt) > 1000) {
	if (((int32_t)(uc_out[BOOM_INDEX]->encoder[0]) - last_body_cnt) < 0)
	    body_angle_cnt_base += 0x1FFFF;
	else
	    body_angle_cnt_base -= 0x1FFFF;
    }
    last_body_cnt = (int32_t)(uc_out[BOOM_INDEX]->encoder[0]);

    c_in->body_angle = (((float)((int32_t)(uc_out[BOOM_INDEX]->encoder[0])+body_angle_cnt_base))-BOOM_TILT_OFFSET)*BOOM_RAD_PER_CNT*BOOM_TILT_GEAR_RATIO;
//    c_in->body_angle = ((float)(body_angle_cnt_base)-BOOM_TILT_OFFSET)*BOOM_RAD_PER_CNT*BOOM_TILT_GEAR_RATIO;
    c_in->body_angle_vel = (c_in->body_angle - last_body_angle) / ((float)(current_medulla_time[BOOM_INDEX]-last_medulla_time[BOOM_INDEX]) * SEC_PER_CNT1 );
    last_body_angle = c_in->body_angle;

    c_in->body_pitch = (((float)(uc_out[BOOM_INDEX]->encoder[2]))-BOOM_PITCH_OFFSET)*BOOM_PITCH_RAD_PER_CNT*BOOM_PITCH_GEAR_RATIO;
    c_in->body_pitch_vel = (c_in->body_pitch - last_body_pitch) / ((float)(current_medulla_time[BOOM_INDEX]-last_medulla_time[BOOM_INDEX]) * SEC_PER_CNT1 );
    //printk("%d - %d = %d\n",(int)(c_in->body_pitch*10000),(int)(last_body_pitch*10000),(int)((c_in->body_pitch - last_body_pitch)*10000));
    last_body_pitch = c_in->body_pitch;

    c_in->zPosition = BOOM_PIVOT_HEIGHT + BOOM_LENGTH*sin(c_in->body_angle) + BOOM_ROBOT_OFFSET;
    c_in->zVelocity = (c_in->zPosition - last_zPosition) / ((float)(current_medulla_time[BOOM_INDEX]-last_medulla_time[BOOM_INDEX]) * SEC_PER_CNT1 );
    last_zPosition = c_in->zPosition;

    if (ABS((int32_t)(uc_out[BOOM_INDEX]->encoder[1]) - last_xPositionEncoder) > 10000) {   // Has the encoder wrapped around?
    //printk("ABS(%d - %d) = %d\n",uc_out[BOOM_INDEX]->encoder[1] , last_xPositionEncoder, ABS(uc_out[BOOM_INDEX]->encoder[1] - last_xPositionEncoder));
    if (((int32_t)(uc_out[BOOM_INDEX]->encoder[1]) - last_xPositionEncoder) > 0)
        xPositionBase -= ((2*PI)*BOOM_HOPPING_RADIUS*BOOM_PAN_GEAR_RATIO);
    else
        xPositionBase += ((2*PI)*BOOM_HOPPING_RADIUS*BOOM_PAN_GEAR_RATIO);
   }

    last_xPositionEncoder = (int32_t)(uc_out[BOOM_INDEX]->encoder[1]);
    c_in->xPosition = (((float)(uc_out[BOOM_INDEX]->encoder[1]))*BOOM_RAD_PER_CNT*BOOM_HOPPING_RADIUS*BOOM_PAN_GEAR_RATIO) + xPositionBase;
    c_in->xVelocity = (c_in->xPosition - last_xPosition) / ((float)(current_medulla_time[BOOM_INDEX]-last_medulla_time[BOOM_INDEX]) * SEC_PER_CNT1 );
    last_xPosition = c_in->xPosition;
    */

    // clear motor torques, this might help with the "ghost" problem. 
    c_out->motor_torqueA = 0.0;
    c_out->motor_torqueB = 0.0;
    c_out->motor_torque_hip = 0.0;
    // Controller update.
    control_switcher_state_machine( c_in, c_out,
        &shm->controller_state, &shm->controller_data[shm->control_index] );

    // Clamp the motor torques.
    c_out->motor_torqueA = CLAMP(c_out->motor_torqueA, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);
    c_out->motor_torqueB = CLAMP(c_out->motor_torqueB, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);
    c_out->motor_torque_hip = CLAMP(c_out->motor_torque_hip, MTR_MIN_HIP_TRQ_LIMIT, MTR_MAX_HIP_TRQ_LIMIT);

    uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            c_out->motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            -c_out->motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    uc_in[HIP_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            c_out->motor_torque_hip, MTR_MIN_HIP_TRQ, MTR_MAX_HIP_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);


    uc_in[A_INDEX]->counter++;
    uc_in[B_INDEX]->counter++;
    uc_in[HIP_INDEX]->counter++;
    //uc_in[BOOM_INDEX]->counter++;

    for (i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++)
	last_medulla_time[i] = uc_out[i]->timestep;

    // Increment and rollover i/o index for datalogging.
    shm->io_index = (++shm->io_index) % SHM_TO_USPACE_ENTRIES;

    for (i = 0; i > NUM_OF_MEDULLAS_ON_ROBOT; i++) {
	if (uc_out[i]->state == 5)   // If the medulla is in error
	    return STATE_ERROR;
    }

    return STATE_RUN;
}

/*****************************************************************************/

unsigned char state_error( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
    // Send zero motor torques.
    uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
        0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
        0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    uc_in[HIP_INDEX]->MOTOR_TORQUE = DISCRETIZE(
        0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);


    // No coming back from this one yet.
    return STATE_ERROR;
}

