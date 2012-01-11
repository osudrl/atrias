/**
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

#define STATE_INIT          0
#define STATE_START         1
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

unsigned short int  last_tranA_cnt;
unsigned short int  last_tranB_cnt;

float               last_motor_angleA = 0.;
float               last_motor_angleB = 0.;
float               last_leg_angleA   = 0.;
float               last_leg_angleB   = 0.;

float               hor_vel;
float               hor_vel_buffer[HOR_VEL_WINDOW];
int                 hor_vel_index     = 0;

float               leg_angle;
float               leg_length;

static unsigned char command = CMD_RUN;

ControllerInput*    c_in;
ControllerOutput*   c_out;

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
    printk("Thermistor[0]: %d, %d\n", (uc_out[B_INDEX]->thermistor[0]), (int)(ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[0])));
    printk("Thermistor[1]: %d, %d\n", (uc_out[B_INDEX]->thermistor[1]), (int)(ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[1])));
    printk("Thermistor[2]: %d, %d\n", (uc_out[B_INDEX]->thermistor[2]), (int)(ADC_TO_TEMP(uc_out[B_INDEX]->thermistor[2])));
	
    printk("Motor Voltage: %d\n", (int)(POWER_ADC_TO_V(uc_out[B_INDEX]->motor_power)));
    printk("Logic Voltage: %d\n", (int)(POWER_ADC_TO_V(uc_out[B_INDEX]->logic_power)));

    printk("Motor Encoder A: %d\n",uc_out[A_INDEX]->TRANS_ANGLE);
    printk("Motor Encoder B: %d\n",uc_out[B_INDEX]->TRANS_ANGLE);
    printk("Leg Encoder A: %d\n",uc_out[A_INDEX]->LEG_SEG_ANGLE);
    printk("Leg Encoder B: %d\n",uc_out[B_INDEX]->LEG_SEG_ANGLE);
    // Keep a copy of the states in memory.
    static unsigned char last_state = STATE_INIT;
    static unsigned char next_state = STATE_INIT;

    switch ( next_state )
    {
        case STATE_INIT:
            next_state = state_wakeup( uc_in, uc_out, last_state );
            last_state = STATE_INIT;
            break;
        case STATE_START:
            next_state =  state_initialize(uc_in, uc_out, last_state);
            last_state = STATE_START;
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
        next_state =  STATE_INIT;

    // Handle controller data change requests.
    if ( shm->req_switch )
    {
        shm->control_index     = 1 - shm->control_index;
        shm->req_switch     = false;
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

    // Check to see if Medullas are awake by sending them a bad command (0).
    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
    {
        uc_in[i]->command = 1;
    }
    
    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ ) {
        // If a Medulla does not report the bad command, then fail.
        if ( uc_out[i]->state != 1)
        {
            rt_printk( "Medulla %u is not in state INIT\n", i );
            return STATE_INIT;
        }
    }

    // If successful, Medullas are ready to receive restarts.
    return STATE_START;
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
        uc_in[i]->command = 2;
    }
    
    for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ ) {
        // If a Medulla does not report the bad command, then fail.
        if ( uc_out[i]->state != 2)
        {
            rt_printk( "Medulla %u is not in state RUN\n", i );
            return STATE_START;
        }
    }

    // If successful, Medullas are ready to receive restarts.
    return STATE_RUN;

}

/*****************************************************************************/

unsigned char state_run( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
    int i;

    // Create pointers to controller I/O
    c_in    = &shm->controller_input[shm->io_index];
    c_out     = &shm->controller_output[shm->io_index];

    // Generate controller input
    //c_in->leg_angleA       = LEG_A_ENC_TO_ANGLE(uc_out[A_INDEX]->LEG_SEG_ANGLE,LEG_A_CALIB_VAL);
    //c_in->leg_angleB       = LEG_B_ENC_TO_ANGLE(uc_out[B_INDEX]->LEG_SEG_ANGLE,LEG_B_CALIB_VAL);
    //c_in->motor_angleA     = LEG_A_ENC_TO_ANGLE(uc_out[A_INDEX]->TRANS_ANGLE,TRAN_A_CALIB_VAL);
    //c_in->motor_angleB     = LEG_B_ENC_TO_ANGLE(uc_out[B_INDEX]->TRANS_ANGLE,TRAN_B_CALIB_VAL);
    c_in->leg_angleA        = UNDISCRETIZE(
        uc_out[A_INDEX]->LEG_SEG_ANGLE,
        MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

    c_in->leg_angleB        = UNDISCRETIZE(
        uc_out[B_INDEX]->LEG_SEG_ANGLE,
        MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

    c_in->motor_angleA     = UNDISCRETIZE(
        tranA_off + uc_out[A_INDEX]->TRANS_ANGLE,
        MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT) + TRAN_A_OFF_ANGLE;

    c_in->motor_angleB     = UNDISCRETIZE(
        tranB_off + uc_out[B_INDEX]->TRANS_ANGLE,
        MIN_TRAN_B_ANGLE, MAX_TRAN_B_ANGLE, MIN_TRAN_B_COUNT, MAX_TRAN_B_COUNT) + TRAN_B_OFF_ANGLE;    
    c_in->motor_velocityA = (c_in->motor_angleA - last_motor_angleA)
        / ( (float)uc_out[A_INDEX]->timestep * SEC_PER_CNT );
    c_in->motor_velocityB = (c_in->motor_angleB - last_motor_angleB) 
        / ( (float)uc_out[B_INDEX]->timestep * SEC_PER_CNT );

    c_in->leg_velocityA = (c_in->leg_angleA - last_leg_angleA)
        / ( (float)uc_out[A_INDEX]->timestep * SEC_PER_CNT );    
    c_in->leg_velocityB = (c_in->leg_angleB - last_leg_angleB)
        / ( (float)uc_out[B_INDEX]->timestep * SEC_PER_CNT );                   

    c_in->toe_switch = uc_out[B_INDEX]->toe_switch;

    last_motor_angleA = c_in->motor_angleA;
    last_motor_angleB = c_in->motor_angleB;
    last_leg_angleA        = c_in->leg_angleA;
    last_leg_angleB        = c_in->leg_angleB;            

    c_in->command = shm->controller_data[shm->control_index].command;

    // Controller update.
    control_switcher_state_machine( c_in, c_out,
        &shm->controller_state, &shm->controller_data[shm->control_index] );
    // Clamp the motor torques.
    c_out->motor_torqueA = CLAMP(c_out->motor_torqueA, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);
    c_out->motor_torqueB = CLAMP(c_out->motor_torqueB, MTR_MIN_TRQ_LIMIT, MTR_MAX_TRQ_LIMIT);
    
    uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            c_out->motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
            -c_out->motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
    
    uc_in[A_INDEX]->counter++;
    uc_in[B_INDEX]->counter++;


    if ((uc_out[A_INDEX]->state == 5) || (uc_out[B_INDEX]->state == 5))
    return STATE_ERROR;
    
    // Increment and rollover i/o index for datalogging.
    shm->io_index = (++shm->io_index) % SHM_TO_USPACE_ENTRIES;

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

    // No coming back from this one yet.
    return STATE_ERROR;
}
