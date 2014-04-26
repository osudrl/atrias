#include <stdio.h>
#include <crc.h>

int main(void)
{
	crc_generate_table();
	printf("CRC %08x compare result: %1x\n", crc_calc("123456789", 9), is_packet_good(0xcbf43926, crc_calc("123456789", 9)));

	return 0;
}

