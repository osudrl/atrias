#include <atrias_robot/lut.h>
 	
float linear_interp(LUT_2dof *lut, float xin, float yin)
{
	// Find the nearest neighbors in the LUT.
	int xi = (int)( (xin - lut->xmin) / lut->xstep);
	int yi = (int)( (yin - lut->ymin) / lut->ystep);

	xi = MIN(lut->Nx - 2, MAX(0, xi));
	yi = MIN(lut->Ny - 2, MAX(0, yi));

	return (lut->z[xi][yi] * (lut->x[xi + 1] - xin) * (lut->y[yi + 1] - yin)
		+ lut->z[xi + 1][yi] * (xin - lut->x[xi]) * (lut->y[yi + 1] - yin)
		+ lut->z[xi][yi + 1] * (lut->x[xi + 1] - xin) * (yin - lut->y[yi])
		+ lut->z[xi + 1][yi + 1] * (xin - lut->x[xi]) * (yin - lut->y[yi]) )
		/ (lut->x[xi + 1] - lut->x[xi]) / (lut->y[yi + 1] - lut->y[yi]);
}


void release_memory(LUT_2dof* lut)
{
	FREE(lut->x);
	FREE(lut->y);
	FREE(lut->z);
	FREE(lut);
}
