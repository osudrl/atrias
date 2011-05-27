#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <drl_library/discretize.h>

int main(int argc, char **argv)
{
	/*float val = atof(argv[1]);
	float min = atof(argv[2]);
	float max = atof(argv[3]);

	uint16_t dis16 = DISCRETIZE_16(val, min, max);
	uint32_t dis32 = DISCRETIZE_32(val, min, max);
	
	printf("val = %.3f, min = %.3f, max = %.3f\n\tdis16 = %u, dis32 = %u\n\n", val, min, max, dis16, dis32); */

	uint8_t dis8	 = atoi(argv[1]);
	uint16_t dis16 = atoi(argv[2]);
	uint32_t dis32 = MAX_32BIT_UINT; //atoi(argv[3]);
	float min = atof(argv[4]);
	float max = atof(argv[5]);

	float val8	= UNDISCRETIZE_8(dis8, min, max);
	float val16 = UNDISCRETIZE_16(dis16, min, max);
	float val32 = UNDISCRETIZE_32(dis32, min, max);
	
	printf("val8 = %.3f, val16 = %.3f, val32 = %.3f\n\n", val8, val16, val32);

	printf("%u\n\n", dis32);

	printf("%.3f    ?     %.3f\n\n", (float)dis32, (float)MAX_32BIT_UINT);

	printf("%.3f\n\n", (float)dis32 / (float)MAX_32BIT_UINT);
}
