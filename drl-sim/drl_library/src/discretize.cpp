#include <drl_library/discretize.h>

uint16_t discretize(float val, float min, float max)
{
	return (uint16_t)((val - min) / (max - min) * MAX_16BIT_UINT);
}

float undiscretize(uint32_t dis, float min, float max)
{
	return (float)dis / (float)MAX_16BIT_UINT * (max - min) + min;
}
