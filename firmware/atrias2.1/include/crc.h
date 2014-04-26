#ifndef _CRC_H
#define _CRC_H

#include "/usr/avr/include/stdio.h"
//#include <stdio.h>
//#include <stdint.h>

typedef uint32_t crc_t;   /* KVH IMU has 32-bit CRC */
crc_t crc_table[256];

crc_t reflect(crc_t crc, uint8_t num_bits);
void crc_generate_table(void);
crc_t crc_calc(uint8_t *packet, uint8_t num_bytes);
uint8_t is_packet_good(crc_t my_crc, crc_t kvh_crc);

#endif /* _CRC_H */

