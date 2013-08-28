#include <medulla_ahrs_math.h>

float sine(float x)
{
	const float B = 4/M_PI;
	const float C = -4/(M_PI*M_PI);
	while (x > M_PI) x -= 2*M_PI;
	while (x < -M_PI) x += 2*M_PI;
	float y = B * x + C * x * ABS(x);
	const float P = 0.225;
	y = P * (y * ABS(y) - y) + y;
	return y;
}

float cosine(float x)
{
	return sine(x+M_PI/2);
}

float msqrt(float x)
{
	union {
		float f;
		int i;
	} tmp;
	tmp.f = x;
	tmp.i = 0x5f3759df - (tmp.i >> 1);
	float y = tmp.f;
	y = y * (1.5f-0.5f*x*y*y);
	return 1.0f/(y*(1.5f-0.5f*x*y*y));
}

float minvsqrt(float x)
{
	union {
		float f;
		int i;
	} tmp;
	tmp.f = x;
	tmp.i = 0x5f3759df - (tmp.i >> 1);
	float y = tmp.f;
	y = y * (1.5f-0.5f*x*y*y);
	return y*(1.5f-0.5f*x*y*y);
}

float arctan2(float y, float x)
{
	float coeff_1 = M_PI / 4.0;
	float coeff_2 = 3.0 * coeff_1;
	float abs_y = ABS(y);
	float angle;

	if (x >= 0.0) {
		float r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	}
	else {
		float r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}

	return y < 0.0 ? -angle : angle;
}

float v_dotp(float v1[3], float v2[3])
{
	static float output;

	output = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];

	return output;
}

void v_crossp(float v1[3], float v2[3], float vOut[3])
{
	static float output[3];

	output[0] = (v1[1]*v2[2]) - (v1[2]*v2[1]);
	output[1] = (v1[2]*v2[0]) - (v1[0]*v2[2]);
	output[2] = (v1[0]*v2[1]) - (v1[1]*v2[0]);

	uint8_t i;
	for (i=0; i<3; i++) {
		vOut[i] = output[i];
	}
}

void v_scale(float v[3], float scale, float vOut[3])
{
	static uint8_t i;
	for (i=0; i<3; i++) {
		vOut[i] = v[i] * scale; 
	}
}

void v_add(float v1[3], float v2[3], float vOut[3])
{
	static uint8_t i;
	for (i=0; i<3; i++) {
		 vOut[i] = v1[i] + v2[i];
	}
}

float v_mod(float v[3])
{
	static float output;
	output = v[0] * v[0];
	output += v[1] * v[1];
	output += v[2] * v[2];
	output = msqrt(output);
	return output;
}

float v_norm(float v[3])
{
	static float mod;
	mod = v_mod(v);
	v[0] /= mod;
	v[1] /= mod;
	v[2] /= mod;
	return mod;
}

void m_init_identity(float m[3][3])
{
	static uint8_t i, j;
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			m[i][j] = (i==j) ? 1 : 0;
		}
	}
}

void m_transpose(float mIn[3][3], float mOut[3][3])
{
	static uint8_t i, j;
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			mOut[j][i] = mIn[i][j];
		}
	}
}

void m_product(float m1[3][3], float m2[3][3], float mOut[3][3])
{
	static float tmp[3];
	static uint8_t i, j, k;
	for (i=0; i<3; i++) {
		for(j=0; j<3; j++) {
			for(k=0; k<3; k++) {
				tmp[k] = m1[i][k] * m2[k][j];
			}
			mOut[i][j] = tmp[0] + tmp[1] + tmp[2];
		}
	}
}

void m_add(float m1[3][3], float m2[3][3], float mOut[3][3])
{
	static uint8_t i, j;
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			mOut[i][j] = m1[i][j] + m2[i][j];
		}
	}
}

int m_inverse(int n, float mat[3][3])
{
	/**
	 * mat = input matrix matrix AND result matrix
	 * n = number of rows = number of columns in mat (n x n)
	 */
	int pivrow;             /* keeps track of current pivot row */
	int k,i,j;              /* k: overall index along diagonal; i: row index; j: col index */
	int pivrows[n]; /* keeps track of rows swaps to undo at end */
	float tmp;              /* used for finding max value and making column swaps */

	for (k = 0; k < n; k++) {
		/* find pivot row, the row with biggest entry in current column */
		tmp = 0;
		for (i = k; i < n; i++) {
			if (ABS(mat[i][k]) >= tmp) {   /* Avoid using other functions inside abs()? */
				tmp = ABS(mat[i][k]);
				pivrow = i;
			}
		}

		/* check for singular matrix */
		if (mat[pivrow][k] == 0.0f) {
			/*Inversion failed due to singular matrix */
			return 0;
		}

		/* Execute pivot (row swap) if needed */
		if (pivrow != k) {
			/* swap row k with pivrow */
			for (j = 0; j < n; j++) {
				tmp = mat[k][j];
				mat[k][j] = mat[pivrow][j];
				mat[pivrow][j] = tmp;
			}
		}
		pivrows[k] = pivrow;   /* record row swap (even if no swap happened) */
		tmp = 1.0f/mat[k][k];   /* invert pivot element */
		mat[k][k] = 1.0f;   /* This element of input matrix becomes result matrix */

		/* Perform row reduction (divide every element by pivot) */
		for (j = 0; j < n; j++) {
			mat[k][j] = mat[k][j]*tmp;
		}

		/* Now eliminate all other entries in this column */
		for (i = 0; i < n; i++) {
			if (i != k) {
				tmp = mat[i][k];
				mat[i][k] = 0.0f;   /* The other place where in matrix becomes result mat */
				for (j = 0; j < n; j++) {
					mat[i][j] = mat[i][j] - mat[k][j]*tmp;
				}
			}
		}
	}

	/* Done, now need to undo pivot row swaps by doing column swaps in reverse order */
	for (k = n-1; k >= 0; k--) {
		if (pivrows[k] != k) {
			for (i = 0; i < n; i++) {
				tmp = mat[i][k];
				mat[i][k] = mat[i][pivrows[k]];
				mat[i][pivrows[k]] = tmp;
			}
		}
	}
	return 1;
}

