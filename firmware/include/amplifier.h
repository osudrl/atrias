#ifndef AMPLIFIER_H
#define AMPLIFIER_H

#include <stdint.h>
#include <util/delay.h>

#include "dzralte_comm.h"
#include "uart.h"
#include "pwm.h"
#include "io_pin.h"

// Sequence for each message type
#define AMP_GET_WRITE_ACCESS_63 0
#define AMP_GET_WRITE_ACCESS_62 1
#define AMP_ENABLE_63 2
#define AMP_ENABLE_62 3
#define AMP_DISABLE_63 4
#define AMP_DISABLE_62 5
#define AMP_GET_CURRENT_63 6
#define AMP_GET_CURRENT_62 7
#define AMP_CHANGE_BAUD_63 8
#define AMP_CHANGE_BAUD_62 9

/** @brief Initilizes the amplifier outputs
 *
 *  @param second_amp If second_amp is true, then this sets parameters for a second amplifier with address 62
 */
void initilize_amp(bool second_amp, int16_t *amp_63_current, int16_t *amp_62_current);
void enable_amp(bool second_amp);
void disable_amp(bool second_amp);
void enable_pwm();
void disable_pwm();
void set_amp_output(int32_t value);
void send_current_read(bool second_amp);
bool check_current_read(bool second_amp);

#endif //AMPLIFIER_H
