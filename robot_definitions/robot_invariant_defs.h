#ifndef ROBOT_INVARIANT_DEFS_H
#define ROBOT_INVARIANT_DEFS_H

typedef enum {
	medulla_state_idle     = 0,
	medulla_state_init     = 1,
	medulla_state_run      = 2,
	medulla_state_stop     = 3,
	medulla_state_halt     = 4,
	medulla_state_error    = 5,
	medulla_state_reset    = 6,
	medulla_state_watchdog = 7
} medulla_state_t;

// EtherCAT IDs
#define MEDULLA_VENDOR_ID                                                  0x60F
#define MEDULLA_LEG_PRODUCT_CODE                                      0x00000001
#define MEDULLA_HIP_PRODUCT_CODE                                      0x00000002
#define MEDULLA_BOOM_PRODUCT_CODE                                     0x00000003
#define MEDULLA_TEST_PRODUCT_CODE                                     0x00000004
#define KPA_SLAVE_CARD_PRODUCT_CODE                                   0x00000005
#define MEDULLA_ASSIGN_ACTIVATE_WORD                                      0x0300

// Total size of process data in each direction for each Medulla type
#define MEDULLA_LEG_INPUTS_SIZE                                               47
#define MEDULLA_LEG_OUTPUTS_SIZE                                               7
#define MEDULLA_HIP_INPUTS_SIZE                                               27
#define MEDULLA_HIP_OUTPUTS_SIZE                                               7
#define MEDULLA_BOOM_INPUTS_SIZE                                              24
#define MEDULLA_BOOM_OUTPUTS_SIZE                                              3

// Number of PDO entries in each direction for each Medulla type
#define MEDULLA_LEG_TX_PDO_COUNT                                              24
#define MEDULLA_LEG_RX_PDO_COUNT                                               3
//#define MEDULLA_HIP_TX_PDO_COUNT                                              31
#define MEDULLA_HIP_TX_PDO_COUNT                                              15
#define MEDULLA_HIP_RX_PDO_COUNT                                               3
#define MEDULLA_BOOM_TX_PDO_COUNT                                             11
#define MEDULLA_BOOM_RX_PDO_COUNT                                              2

// Medulla IDs
#define MEDULLA_ID_PREFIX_MASK                                              0x30
#define MEDULLA_LEG_ID_PREFIX                                               0x00
#define MEDULLA_HIP_ID_PREFIX                                               0x10
#define MEDULLA_BOOM_ID_PREFIX                                              0x20
#define MEDULLA_TEST_ID_PREFIX                                              0x30

#define MEDULLA_AMPLIFIER_DEBUG                                             0x00
#define MEDULLA_LEFT_LEG_A_ID                       (MEDULLA_LEG_ID_PREFIX  + 1)
#define MEDULLA_LEFT_LEG_B_ID                       (MEDULLA_LEG_ID_PREFIX  + 2)
#define MEDULLA_LEFT_HIP_ID                         (MEDULLA_HIP_ID_PREFIX  + 0)
#define MEDULLA_RIGHT_LEG_A_ID                      (MEDULLA_LEG_ID_PREFIX  + 3)
#define MEDULLA_RIGHT_LEG_B_ID                      (MEDULLA_LEG_ID_PREFIX  + 4)
#define MEDULLA_RIGHT_HIP_ID                        (MEDULLA_HIP_ID_PREFIX  + 1)
#define MEDULLA_BOOM_ID                             (MEDULLA_BOOM_ID_PREFIX + 0)

// Limit switch masks for medullas
#define MEDULLA_LLEG_ASIDE_LSW_MASK                    0b00111111
#define MEDULLA_LLEG_BSIDE_LSW_MASK                    0b00011111
#define MEDULLA_RLEG_ASIDE_LSW_MASK                    0b00111111
#define MEDULLA_RLEG_BSIDE_LSW_MASK                    0b00011111

// Medulla errors
typedef enum {
	medulla_error_estop         = 1<<0,
	medulla_error_limit_switch  = 1<<1,
	medulla_error_thermistor    = 1<<2,
	medulla_error_motor_voltage = 1<<3,
	medulla_error_logic_voltage = 1<<4,
	medulla_error_encoder       = 1<<5,
	medulla_error_halt          = 1<<6,
	medulla_error_amplifier     = 1<<7
} medulla_error_t;

// Safety cut off values

// Danger region for motor power
#define MOTOR_VOLTAGE_DANGER_MAX                                            3000
#define MOTOR_VOLTAGE_DANGER_MIN                                            1500

// Low voltage cut off for logic power
#define LOGIC_VOLTAGE_MIN                                                   2950

/** @brief Maximum allowable thermistor value
  * This is a minimum ADC value, but it
  * relates to the maximum temp.
  */
#define THERMISTOR_MAX_VAL                                                   300

// Loop period for main RT operations
#define CONTROLLER_LOOP_PERIOD_NS                                      1000000LL
/** @brief The offset between the DC and our code loop.
  */
#define CONTROLLER_LOOP_OFFSET_NS                                       300000LL

//Loop period for the GUI
#define GUI_LOOP_PERIOD_NS                                            20000000LL

#define LEG_A_CALIB_LOC                                       1.3089969389957472
#define LEG_A_MOTOR_MIN_LOC                                         -0.305432619
#define LEG_A_MOTOR_MAX_LOC                                           2.35619449

#define LEG_B_CALIB_LOC                                       1.8325957145940461
#define LEG_B_MOTOR_MIN_LOC                                          0.785398163
#define LEG_B_MOTOR_MAX_LOC                                           3.44702527

#define LEFT_HIP_MOTOR_MIN_LOC                                               0.0   // TODO: Get actual value.
#define LEFT_HIP_MOTOR_MAX_LOC                                               0.0   // TODO: Get actual value.
#define RIGHT_HIP_MOTOR_MIN_LOC                                              0.0   // TODO: Get actual value.
#define RIGHT_HIP_MOTOR_MAX_LOC                                              0.0   // TODO: Get actual value.

// The minimum and maximum distance in positions between the motors. For leg length
// protection.
#define LEG_LOC_DIFF_MIN                                                     0.0
#define LEG_LOC_DIFF_MAX                                              2.650929045

#define LEG_LOC_SAFETY_DISTANCE                                       0.34906585
#define HIP_LOC_SAFETY_DISTANCE                                              0.2   // TODO: This is a fake value!

// The maximum acceptable change in encoder value in a single cycle, for detecting
// erroneous encoder readings.
#define MAX_ACCEPTABLE_ENCODER_CHANGE                                    5000000

// Acceleration of motor output in radians/s^2/amp
#define ACCEL_PER_AMP                                                  1.7185879

/** @brief The maximum commanded amplifier value.
  * This is the maximum value sent to the
  * Medullas for the amplifier command.
  */
#define MTR_MAX_COUNT                                                      19900

// Limits the maximum adjustment that may be applied to the leg position when the spring deflections
// are zeroed during initialization (in radians).
#define MAX_LEG_POS_ADJUSTMENT                                               0.1

/** @brief The voltage at which the xMega's ADCs max out.
  * This is used for scaling the ADC values.
  */
#define MEDULLA_ADC_MAX_VOLTS                                               2.70

/** @brief The amount of time RT Ops should wait for the Medullas to enter idle after a reset.
  */
#define MEDULLA_RESET_TIME_MS                                                  5

/** @brief This is the number of radians represented by one tick of the motor's internal incremental encoder.
  * Used for scaling.
  */
#define INC_ENC_RAD_PER_TICK                                 0.00000897597901026

/** @brief This is the rate at which the Medulla's timers run. Used for timestamp usage for velocity calculation.
  */
#define MEDULLA_TIMER_FREQ                                            32000000.0

/** @brief The number of bits reported by the boom encoders.
  * For rollover compensation
  */
#define BOOM_ENCODER_BITS                                                     17

/** @brief The number of radians rotated when the pitch encoder moves 1 tick.
  */
#define PITCH_ENCODER_RAD_PER_TICK                                 0.00002396844

/** @brief The number of radians rotated when the Z encoder moves 1 tick.
  */
#define BOOM_Z_ENCODER_RAD_PER_TICK                -0.00000684812851734661263267

/** @brief The direction for the left hip motor.
  */
#define LEFT_MOTOR_HIP_DIRECTION                                             1.0

/** @brief The direction for the right hip motor.
  */
#define RIGHT_MOTOR_HIP_DIRECTION                                           -1.0

/** @brief The radians per tick for the hip's incremental encoder.
  */
#define HIP_INC_ENCODER_RAD_PER_TICK                  0.000009889017906384604997

/** @brief The radians per tick for the hip's absolute encoder.
  */
#define HIP_ABS_ENCODER_RAD_PER_TICK                  0.000766990393942820614859

#endif // ROBOT_INVARIANT_DEFS_H
