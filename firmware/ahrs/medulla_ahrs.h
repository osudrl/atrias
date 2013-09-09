#ifndef MEDULLA_AHRS_H
#define MEDULLA_AHRS_H

#include <medulla_kvh1750.h>
#include <medulla_ahrs_math.h>

#include <uart.h>

// #define ACC_WEIGHT 0.080   /* Accelerometer weight relative to gyro's weight of 1 */
// #define ACC_SCALE_WEIGHT 2   /* Gradually drive accelerometer weight to zero. For example, a value of 5 here will drive the accelerometer weight to zero if the magnitude of the measured acceleration differs by more than 1/5 gravity. */

/**
 * @brief Set up the AHRS.
 */
void setup_ahrs(void);

/**
 * @brief Update the AHRS.
 * @param dt Time interval since last update.
 * @output dcm_out Output DCM.
 */
void update_ahrs(float dt, float dcm_out[3][3]);

/**
 * @brief Output debug string.
 * @output buffer Buffer to populate with debug string.
 */
void debug_ahrs(uint8_t *buffer);

#endif /* MEDULLA_AHRS_H */

