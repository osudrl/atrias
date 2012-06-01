/**
 * @file
 * @author Devin Koepl   
 * @brief Define states, error codes, special values, and medual communication structs. 
 *
 */

#ifndef FUNCS_H_UCONTROLLER
#define FUNCS_H_UCONTROLLER

// Medulla IDs
#define MEDULLA_A_ID 				0b0001
#define MEDULLA_B_ID 				0b0010
#define MEDULLA_HIP_ID 				0b1000
#define MEDULLA_BOOM_ID				0b1001
#define MEDULLA_LEG_BM				0b1000

/*
 * These are still being used by all_in_one_controller_wrapper.cpp
 * Can't build the GUI without these macros. -CJM
 */
#define CMD_RESTART                             (1<<0)
#define CMD_DISABLE                             (1<<1)
#define CMD_RUN                                 (1<<2)

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
// Command bytes.
// During normal running, the controller should switch between sending the two run mode bytes
// as its command byte at each timestep.  We can implement watchdog timers to detect if the
// control computer is operating correctly.  CMD_BAD is a place-holder so the value is never used.
#define CMD_BAD                                 0
#define CMD_RESTART                             (1<<0)
#define CMD_DISABLE                             (1<<1)
#define CMD_RUN                                 (1<<2)
#define CMD_RESET                               (1<<3)
#define CMD_RUN_TOGGLE_bm                       (1<<6)
#define CMD_RUN_OK_bm                           (1<<7)

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
#define RAD_PER_CNT				(4.90451647e-9)

// Medulla A sensors:           
#define LEG_A_CALIB_LOC                         -1.308996694
#define LEG_A_TRAN_ENC_TO_ANGLE(val) 		LEG_A_CALIB_LOC + (val-TRAN_A_CALIB_VAL)*TRAN_A_RAD_PER_CNT
#define LEG_A_LEG_ENC_TO_ANGLE(val) 		LEG_A_CALIB_LOC + (val-LEG_A_CALIB_VAL)*LEG_A_RAD_PER_CNT

#define LEG_A_CALIB_VAL                         352351997.0
#define LEG_A_RAD_PER_CNT			4.9016592164112157e-09
#define TRAN_A_CALIB_VAL                        284287500.0
#define TRAN_A_RAD_PER_CNT			4.9009447987843945e-09

#define MEDULLA_A_ENC_MAX			563013448
#define MEDULLA_A_ENC_MIN			124891505

// Medulla B sensors:

#define LEG_B_CALIB_LOC                     	4.450589258
#define LEG_B_TRAN_ENC_TO_ANGLE(val) 		LEG_B_CALIB_LOC - (val-TRAN_B_CALIB_VAL)*TRAN_B_RAD_PER_CNT
#define LEG_B_LEG_ENC_TO_ANGLE(val) 		LEG_B_CALIB_LOC - (val-LEG_B_CALIB_VAL)*LEG_B_RAD_PER_CNT

#define LEG_B_CALIB_VAL                         348884314.0
#define LEG_B_RAD_PER_CNT			4.9117396941444945e-09
#define TRAN_B_CALIB_VAL                        285247647.0
#define TRAN_B_RAD_PER_CNT			4.9113235275589447e-09

#define MEDULLA_B_ENC_MAX			557704303
#define MEDULLA_B_ENC_MIN			123645104

// Defines for incremental encoder
#define INC_ENCODER_RAD_PER_TICK		((2*PI)/542600.0)

// These are the rough ranges of motion for the zero force leg segments.
//  E.g. they are the leg angle counts when the transmission is at the limits
//  and the spring is uncompressed.
#define MIN_TRAN_SEG_A_COUNT		198440000
#define MAX_TRAN_SEG_A_COUNT		513200000
#define MIN_TRAN_SEG_B_COUNT		199168000		
#define MAX_TRAN_SEG_B_COUNT		512698000

// Need to set these during sensor calibration if you want the spring deflections
// to be computed accurately.  Add these offsets to the transmission angle.
#define TRAN_A_OFF_ANGLE			-0.014725
#define TRAN_B_OFF_ANGLE			0.020483

// Hip Medulla sensors:
#define MIN_HIP_COUNT				9704
#define MAX_HIP_COUNT				18806
#define HIP_COUNT_RANGE				9102
#define MIN_HIP_ANGLE				-0.20943951 //157079633
#define MAX_HIP_ANGLE				0.157079633

// Boom Medulla sensors:
// Boom length: 1.858m (not to CoM of robot)
//		2.13m (measuring tape estimate)
// Boom height: 1.00965m
// 		1.108m (measuring tape estimate)
// False floor height: 0.1525m
// Effective boom height: 0.85715m
// 		0.9555 (measuring tape)

#define BOOM_PIVOT_HEIGHT			0.9144
#define	BOOM_LENGTH				1.8796
#define BOOM_ROBOT_OFFSET			0.1303

// Pan (angle about the room):
#define BOOM_PAN_GEAR_RATIO 			(1/10.3439)   // Gear ratio between boom rotation and boom rotation encoder

// Tilt (angle up and down):
#define BOOM_TILT_GEAR_RATIO			(1.0/7.0)
#define BOOM_KNOWN_TILT_ANGLE			0.
#define BOOM_KNOWN_TILT_CNT			61032
#define TILT_CNT_THRESH				30000

#define BOOM_RAD_PER_CNT			((2.0*PI)/131072.0)
#define BOOM_TILT_OFFSET			37300.0
#define BOOM_HOPPING_RADIUS			2.032
#define BOOM_ENC_ROLLOVER_DISTANCE		12.7674

#define BOOM_PITCH_OFFSET			1931
#define BOOM_PITCH_RAD_PER_CNT			0.001533980787886
#define BOOM_PITCH_GEAR_RATIO			0.5

// Roll (robot angle on the boom):
#define BOOM_ROLL_GEAR_RATIO		        1.84615385

// For motors
#define MTR_DIR_bm				(1<<15)
#define MTR_MIN_TRQ				-60.0
#define MTR_MAX_TRQ  				60.0

#define MTR_MIN_HIP_TRQ				-60.0
#define MTR_MAX_HIP_TRQ				60.0

#define MTR_MAX_TRQ_LIMIT			60.0
#define MTR_MIN_TRQ_LIMIT			-60.0

#define MTR_MAX_HIP_TRQ_LIMIT			60.0
#define MTR_MIN_HIP_TRQ_LIMIT			-60

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

#define SEC_PER_CNT1				6.25E-8
#define SEC_PER_CNT				6.25E-8 //1.25E-7

#define ADC_VAL_TO_VOLTAGE(val)                 (val * (2.68/255.0))

#define THERM_VAL_TO_R(val)                     (4700.0/((3.26/(ADC_VAL_TO_VOLTAGE(val))) - 1.0))
#define ADC_TO_TEMP(val)                        ((1.0/( (1.0/298.15) + (1.0/3988.0)*log(THERM_VAL_TO_R(val)/10000))) - 273.15)
#define THERM_MAX_VAL                           39

// Calculates the input voltage to a voltage divider. On the current menial board there is a 30/1 ratio between input and output voltage.
// This also accounts for the small offset in the ADC measurement. 
#define POWER_ADC_TO_V(val)                     ((ADC_VAL_TO_VOLTAGE(val)*30) - 2)

// If the motor voltage is between the max and min values, then we want to disable. We assume that below the minumum voltage we are not
// trying to move the motors, so it's okay if we don't disable. Also, the amps will be disabled so even if we try and move the motor it won't move
#define MOTOR_POWER_DANGER_MAX                  140   // 44 Volts
#define MOTOR_POWER_DANGER_MIN					10   // 3 Volts

// If the logic power is below this level then we should be concerned and disable everything
#define LOGIC_POWER_MIN                         31  // 10 Volts 

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
//	int16_t		ampCurrent;
	
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
