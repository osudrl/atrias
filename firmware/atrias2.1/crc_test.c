#include <stdio.h>
#include <crc.h>

void print_crc_table()
{
	int i;
	for (i=0; i<20; i++) {
		printf("CRC table entry %3d: %8x\n", i, crc_table[i]);
	}
}

int main(void)
{
	crc_generate_table();
	print_crc_table();
	printf("CRC %08x compare result: %1x\n", crc_calc("123456789", 9), is_packet_good(0xcbf43926, crc_calc("123456789", 9)));

	return 0;
}

