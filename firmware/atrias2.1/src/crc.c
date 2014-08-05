#include <crc.h>

/*
 * CRC lookup table generator and calculator adapted from
 * http://www.zorc.breitbandkatze.de/crc.html
 *
 * The KVH 1750 uses all of the header, data, status, sequence number, and
 * temperature in its calculation of the CRC. The following parameters are
 * used (refer to p. 12 of the External Electrical Signaling Interface Control
 * Document):
 *
 *   Width:       32
 *   Poly:        0x04c11db7
 *   Reflect In:  False
 *   XOR In:      0xffffffff
 *   Reflect Out: False
 *   XOR Out:     0x0000
 *
 * For other CRC configurations, refer to the URL above.
 */
crc_t crc_table[256];
const uint8_t CRC_ORDER = 32;
const crc_t CRC_HIGHBIT = (crc_t) (1UL << 31);
const crc_t CRC_POLY = (crc_t) (0x04c11db7);
const crc_t CRC_INIT = (crc_t) (0xffffffff);

void crc_generate_table(void)
{
	/*
	 * Generate CRC lookup table
	 */
	uint8_t i;
	uint16_t dividend;
	crc_t remainder;

	for (dividend=0; dividend<256; dividend++) {
		remainder = (crc_t) dividend;
		remainder <<= CRC_ORDER-8;

		for (i=0; i<8; i++) {
			if (remainder & CRC_HIGHBIT) {;
				remainder = (remainder << 1) ^ CRC_POLY;
			}
			else {
				remainder = (remainder << 1);
			}
		}
		crc_table[dividend] = remainder;
	}
}

crc_t crc_calc(uint8_t *packet, uint8_t num_bytes)
{
	/*
	 * Fast lookup table algorithm without augmented zero bytes, e.g. used in
	 * pkzip. Only usable with polynom orders of 8, 16, 24 or 32.
	 */
	crc_t crc = CRC_INIT;

	/* Divide packet by polynomial one byte at a time. */
	while (num_bytes--) {
		crc = (crc << 8) ^ crc_table[ ((crc >> 24) & 0xff) ^ *packet++];
	}

	return crc;
}

uint8_t is_packet_good(crc_t my_crc, crc_t kvh_crc)
{
	return (my_crc == kvh_crc);
}

void crc_debug_print_table()
{
	int i;
	printf("CRC table entries:\n");
	for (i=0; i<256; i++) {
		printf("%08lx\n", crc_table[i]);
	}
}

void crc_debug_check_crc()
{
	// Test CRC with packet from IMU.
	uint8_t str[] = {0xfe,0x81,0xff,0x55,0xb6,0x06,0xd8,0x1e,0xb6,0xc4,0x94,0x0f,0x36,0xd9,0x5a,0x83,0x3c,0x61,0xad,0x86,0x3b,0xe5,0xd0,0x86,0x3f,0x7f,0xdc,0xd4,0x77,0x72,0x00,0x26};
	uint32_t crc = 0xc0f2d540;
	uint8_t len = 32;

	printf("CRC %08lx compare result: %1x\n", crc_calc(str, len), is_packet_good(crc_calc(str, len), crc));
}

