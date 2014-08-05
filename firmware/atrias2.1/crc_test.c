#include <stdio.h>
#include <crc.h>

int main(void)
{
	crc_generate_table();
	crc_debug_print_table();
	crc_debug_check_crc();

	return 0;
}

