#include "hip_lut.h"

#define LUT_ENTRIES			6
#define MAX_HT					1.0029

float ht_lut[] = {0.7664, 0.7924, 0.8498, 0.9219, 0.9679, 1.0029};

float cnt_lut[] = {4100.*3.9, 4000.*3.9, 3500.*3.9, 3050.*3.9, 2800.*3.9, 2600.*3.9};

extern unsigned short int interpolate_hip_mtr_pos( float ht )
{
	int i = 0;

	if ( ht < MAX_HT )
	{
		// Use linear interpolation to calculate the hip pwm count.

		// First find what values the input is between in the lut.
		while ( ht > ht_lut[i] )
		{
			i++;
		}

		return (unsigned short int) cnt_lut[i - 1] + (ht - ht_lut[i - 1]) * (cnt_lut[i] - cnt_lut[i - 1]) / (ht_lut[i] - ht_lut[i - 1]);
	}

	return CNT_FOR_MAX_HT;
}

