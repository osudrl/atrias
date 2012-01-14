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

// Status bits.
//   If any of the bits in the status byte are set, there is an error in the uController.
//   The following #defines describe the bit locations of each possible error.

#define STATUS_ESTOP				(1<<0)
#define STATUS_LIMITSW				(1<<1)
#define STATUS_OVER_TEMP			(1<<2)
#define STATUS_MOTOR_OUT_OF_RANGE	(1<<3)
#define STATUS_MOTOR_CTRL_DISABLE	(1<<4)
#define STATUS_MOTOR_VOLTAGE_LOW	(1<<5)
#define STATUS_LOGIC_VOLTAGE_LOW	(1<<6)
#define STATUS_ENCODER_ERROR		(1<<7)

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

// 32 Bit Encoder defines
#define ENC_PER_CNT				1

// Medulla A sensors:           
#define LEG_A_CALIB_LOC                         -1.308996694
#define LEG_A_ENC_TO_ANGLE(val,calib) 		LEG_A_CALIB_LOC + (val-calib)*RAD_PER_CNT

#define LEG_A_CALIB_VAL                         354691251.0
#define TRAN_A_CALIB_VAL                        284556173.0

#define MIN_LEG_SEG_A_COUNT			134185219
#define MAX_LEG_SEG_A_COUNT			682343461
#define MIN_LEG_SEG_A_ANGLE			0.296705973
#define MAX_LEG_SEG_A_ANGLE			-2.35619449

#define MAX_TRAN_A_COUNT			617778691
#define MIN_TRAN_A_COUNT			70126263 
#define MIN_TRAN_A_ANGLE			0.296705973
#define MAX_TRAN_A_ANGLE			-2.35619449

#define MEDULLA_A_ENC_MAX			563013448
#define MEDULLA_A_ENC_MIN			124891505

// Medulla B sensors:

#define LEG_B_CALIB_LOC                     	4.450589258
#define LEG_B_ENC_TO_ANGLE(val,calib) 		LEG_B_CALIB_LOC + (calib - val)/ENC_CNT_PER_RAD

#define LEG_B_CALIB_VAL                         349044101.0
#define TRAN_B_CALIB_VAL                        285253415.0


#define MAX_LEG_SEG_B_COUNT			139212755
#define MIN_LEG_SEG_B_COUNT			681805251
#define MIN_LEG_SEG_B_ANGLE			2.84488668
#define MAX_LEG_SEG_B_ANGLE			5.49778714

#define MAX_TRAN_B_COUNT			69387705
#define MIN_TRAN_B_COUNT			611961703
#define MIN_TRAN_B_ANGLE			2.84488668
#define MAX_TRAN_B_ANGLE			5.49778714

#define MEDULLA_B_ENC_MAX			557704303
#define MEDULLA_B_ENC_MIN			123645104

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

#define ADC_VAL_TO_VOLTAGE(val)                 (val * (2.68/255.0))

#define THERM_VAL_TO_R(val)                     (4700.0/((3.26/(ADC_VAL_TO_VOLTAGE(val))) - 1.0))
#define ADC_TO_TEMP(val)                        ((1.0/( (1.0/298.15) + (1.0/3988.0)*log(THERM_VAL_TO_R(val)/10000))) - 273.15)
#define THERM_MAX_VAL                           39

#define POWER_ADC_TO_V(val)                     ((ADC_VAL_TO_VOLTAGE(val)*30) - 2)
#define MOTOR_POWER_MIN                         70
#define LOGIC_POWER_MIN                         35

// For GCC running on a 32-bit machine to minimize the size in memory of these structs, the smallest
// values must come first.  This is important for compatability with the avr-gcc compiled code.

/**
 * @ berief means for talking to the medullas
 */
typedef struct 
{
	uint32_t	enc_max;
	uint32_t	enc_min;
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
