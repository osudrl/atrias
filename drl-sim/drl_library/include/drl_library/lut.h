/* Lookup table (LUT) class.  Uses lookup tables stored as human-readable files of the form:
 	[integer number of dimensions]
 	[number of entries for dimension] => [min] : [stepsize] : [max]
 	...
 	[first LUT entry]
 	[second LUT entry]
 	
 	Lookup table entries go down the rows, across the columns, and finally through pages, i.e. MATLAB's normal indexing scheme.
*/
 	
#ifndef FUNCS_H_LUT
#define FUNCS_H_LUT

typedef struct
{
	int Nx, Ny;

	float xmin, xstep, xmax;
	float ymin, ystep, ymax;

	float* x;
	float* y;               
	float** z;     
} LUT_2dof;

void create2dof_lut(LUT_2dof*);

float linear_interp(LUT_2dof*, float, float);

void release_memory(LUT_2dof*);
 	
#endif // FUNCS_H_LUT
