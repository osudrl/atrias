#ifndef MEDULLA_AHRS_MATH_H
#define MEDULLA_AHRS_MATH_H

#include <stdint.h>

#define M_PI 3.14159265358979323846

#define ABS(x) (((x) < 0) ? -(x) : (x))
#define CLAMP(x, min, max) (((x) < min) ? (min):(((x) > max) ? (max):(x)))
#define MIN(a, b) ((a < b) ? (a) : (b))
#define MAX(a, b) ((a > b) ? (a) : (b))

/* Sine, cosine, sqrt functions from multipilot32. */
float sine(float x);
float cosine(float x);
float msqrt(float x);
float minvsqrt(float x);

/* atan2 */
float arctan2(float y, float x);

/* Dot product */
float v_dotp (float v1[3], float v2[3]);

/* Cross product */
void v_crossp (float v1[3], float v2[3], float vOut[3]);

/* Scalar multiplication */
void v_scale (float v[3], float scale, float vOut[3]);

/* Addition */
void v_add (float v1[3], float v2[3], float vOut[3]);

/* Calculate modulus of vector = sqrt(x^2 + y^2 + z^2) */
float v_mod (float v[3]);

/* Normalize vector to a vector with same direction, mod 1, and return old modulus. */
float v_norm (float v[3]);

/* 3x3 identity matrix initialization */
void m_init_identity (float m[3][3]);

/* 3x3 matrix transpose -- receive matrix mIn as input and output its transpose mOut. */
void m_transpose (float mIn[3][3], float mOut[3][3]);

/* 3x3 matrix multiplication */
void m_product (float m1[3][3], float m2[3][3], float mOut[3][3]);

/* 3x3 matrix addition */
void m_add (float m1[3][3], float m2[3][3], float mOut[3][3]);

/* Matrix Inversion Routine from http://www.arduino.cc/playground/Code/MatrixMath
 * * This function inverts a matrix based on the Gauss Jordan method.
 * * Specifically, it uses partial pivoting to improve numeric stability.
 * * The algorithm is drawn from those presented in 
 *       NUMERICAL RECIPES: The Art of Scientific Computing.
 * * The function returns 1 on success, 0 on failure.
 * * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
 */
int m_inverse(int n, float mat[3][3]);

#endif /* MEDULLA_AHRS_MATH_H */

