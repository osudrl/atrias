/**
 * @file
 * @author Devin Koepl   
 * @brief Define states, error codes, special values, and medual communication structs. 
 *
 */

#ifndef FUNCS_H_UCONTROLLER
#define FUNCS_H_UCONTROLLER

#ifdef _AVR_
#include <stdint.h>
#endif

// Medulla IDs
#define MEDULLA_A_ID 				0xAA
#define MEDULLA_B_ID 				0xBB
#define MEDULLA_HIP_ID 				0xCC
#define MEDULLA_BOOM_ID				0xDD

// Command bytes.
// During normal running, the controller should switch between sending the two run mode bytes
// as its command byte at each timestep.  We can implement watchdog timers to detect if the
// control computer is operating correctly.  CMD_BAD is a place-holder so the value is never used.
#define CMD_BAD					0
#define CMD_RESTART				(1<<0)
#define CMD_DISABLE				(1<<1)
#define CMD_RUN					(1<<2)
#define CMD_RESET				(1<<3)
#define CMD_RUN_TOGGLE_bm			(1<<6)
#define CMD_RUN_OK_bm				(1<<7)

// Hip commands - send these in the HIP_MTR_CMD.
/** @brief command for hip to behave like a pin joint */
#define HIP_CMD_PIN				0xF0

/** @brief command for hip to be rigid */
#define HIP_CMD_RIGID				0x00

// Status bits.
//   If any of the bits in the status byte are set, there is an error in the uController.
//   The following #defines describe the bit locations of each possible error.

/** @breif  The limit switch was hit. */
#define STATUS_LIMITSW				(1<<7)

/** @breif  The step timer rolled over. */
#define STATUS_TCOVF				(1<<6)	

/** @brief  The PWM command was out of range. */
#define STATUS_BADPWM				(1<<5)

/** @breif  There was a problem wit1h the encoders .*/
#define STATUS_ENC				(1<<4)

/** @breif  The command received was invalid, or the toggle bit was wrong. */
#define STATUS_BADCMD				(1<<3)	

/** @breif  Everything is OK, but the uController is in a disabled state */
#define STATUS_DISABLED				(1<<2) 

/** @breif  The uC is taking control of the motor */
#define STATUS_DANGER				(1<<1) 

/** @breif  The motor didn't move as expected */
#define STATUS_BADMOTOR				(1<<0) 

/* Sensor values:
* The Biss-C 32-bit encoders measure the absolute leg segment angles relative to the body, and do not roll over.
* The SSI encoders meausre the motor angles relative to the body and roll over.
*/
#define LEG_SEG_ANGLE				encoder[1]
#define TRANS_ANGLE 				encoder[0]

/** @brief used to access the motor field in uControllerInput */
#define MOTOR_TORQUE				motor_torque

/** @brief used to access the motor field in uControllerInput */
#define HIP_MTR_CMD				motor_torque

#define BOOM_PAN_CNT				enc16[0]
#define BOOM_TILT_CNT				enc16[1]
#define BOOM_ROLL_CNT				enc16[2]					

// Medulla A sensors:
#define MIN_LEG_SEG_A_COUNT			134185219
#define MAX_LEG_SEG_A_COUNT			682343461
#define MIN_LEG_SEG_A_ANGLE			0.296705973
#define MAX_LEG_SEG_A_ANGLE			-2.35619449

#define MAX_TRAN_A_COUNT			617778691
#define MIN_TRAN_A_COUNT			70126263 
#define MIN_TRAN_A_ANGLE			0.296705973
#define MAX_TRAN_A_ANGLE			-2.35619449

// Medulla B sensors:
#define MAX_LEG_SEG_B_COUNT			139212755
#define MIN_LEG_SEG_B_COUNT			681805251
#define MIN_LEG_SEG_B_ANGLE			2.84488668
#define MAX_LEG_SEG_B_ANGLE			5.49778714

#define MAX_TRAN_B_COUNT			69387705
#define MIN_TRAN_B_COUNT			611961703
#define MIN_TRAN_B_ANGLE			2.84488668
#define MAX_TRAN_B_ANGLE			5.49778714

// These are the rough ranges of motion for the zero force leg segments.
//  E.g. they are the leg angle counts when the transmission is at the limits
//  and the spring is uncompressed.
#define MIN_TRAN_SEG_A_COUNT		198440000
#define MAX_TRAN_SEG_A_COUNT		513200000
#define MIN_TRAN_SEG_B_COUNT		199168000		
#define MAX_TRAN_SEG_B_COUNT		512698000

// Need to set these during sensor calibration if you want the spring deflections
// to be computed accurately.  Add these offsets to the transmission angle.
#define TRAN_A_OFF_ANGLE			0.0273115
#define TRAN_B_OFF_ANGLE			0.0265556

// Hip Medulla sensors:
#define MIN_HIP_COUNT				0
#define MAX_HIP_COUNT				7200
#define MIN_HIP_ANGLE
#define MAX_HIP_ANGLE

// Boom Medulla sensors:
// Boom length: 1.858m (not to CoM of robot)
//		2.13m (measuring tape estimate)
// Boom height: 1.00965m
// 		1.108m (measuring tape estimate)
// False floor height: 0.1525m
// Effective boom height: 0.85715m
// 		0.9555 (measuring tape)

#define BOOM_PIVOT_HEIGHT			0.9555
#define	BOOM_LENGTH				2.13

// Pan (angle about the room):
#define BOOM_PAN_GEAR_RATIO 		1

// Tilt (angle up and down):
#define BOOM_TILT_GEAR_RATIO		13
#define BOOM_KNOWN_TILT_ANGLE		0.
#define BOOM_KNOWN_TILT_CNT			61032
#define TILT_CNT_THRESH				30000	

// Roll (robot angle on the boom):
#define BOOM_ROLL_GEAR_RATIO		        1.84615385

// For motors
#define MTR_DIR_bm				(1<<15)
#define MTR_MIN_TRQ				-50.0
#define MTR_MAX_TRQ  				50.0
#define MTR_MAX_TRQ_LIMIT			30.0
#define MTR_MIN_TRQ_LIMIT			-30.0
#define MTR_MIN_CNT				-19900
#define MTR_MAX_CNT 				19900
#define MTR_ZERO_CNT				0
// This should only be just enough torque to hold open the legs.
#define AMPS_OPEN				1

/** @brief macro to hold the open the legs */
#define PWM_OPEN				15 //((uint16_t)(MTR_ZERO_CNT + (AMPS_OPEN / (MTR_MAX_TRQ/(MTR_ZERO_CNT))) ))


// For handling rollover of encoders
#define ROLLOVER13BIT_THRESHOLD 		4096
#define MAX_13BIT				8192
#define ROLLOVER16BIT_THRESHOLD	        	32768
#define MAX_16BIT				65536

#define SEC_PER_CNT				125E-9

#define ADC_VAL_TO_VOLTAGE(val)			(val * (3.26/255.0))
#define THERM_VAL_TO_R(val)			(4700.0/((3.26/(ADC_VAL_TO_VOLTAGE(val))) - 1.0))
//#define ADC_TO_TEMP(val)			((1.0/( (1.0/298.15) + (1.0/3988.0)*log(THERM_V_TO_R(val)/10000))) - 273.15)

// For GCC running on a 32-bit machine to minimize the size in memory of these structs, the smallest
// values must come first.  This is important for compatability with the avr-gcc compiled code.

/**
 * @ berief means for talking to the medullas
 */
typedef struct 
{
        /** @breif tell the motors what to do */
	int32_t	motor_torque;
	uint16_t counter;

	/** @brief tell the medulla what to do */
        uint8_t		command;
} uControllerInput;

typedef struct
{
	uint32_t 	encoder[3];

	uint16_t 	timestep;
	uint16_t	counter;
	
	uint8_t		id;
	uint8_t		state;
	uint8_t		error_flags;
	uint8_t		limitSW;
	uint8_t		toe_switch;
	uint8_t		motor_power;
	uint8_t		logic_power;	
	uint8_t		thermistor[3];
} uControllerOutput;

#endif // FUNCS_H_UCONTROLLER
