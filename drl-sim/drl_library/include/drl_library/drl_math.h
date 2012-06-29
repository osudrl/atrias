// Devin Koepl

#ifndef DRL_MATH_H
#define DRL_MATH_H


#define PI 3.14159265

#define ABS(x)	(((x) < 0) ? -(x) : (x))
#define CLAMP(x, min, max) ( ((x) < min) ? (min):( ((x) > max) ? (max):(x) ) )
#define SIGN(x) (((x) < 0) ? (-1) : (1))

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

/*
 * These are used by a few controllers to calculate leg lengths and angles from
 * the motor angles. I'm putting them here in case they ever need to be
 * changed.
 */
#define DELTA_Y(mtrAngA, mtrAngB) (0.5*sin(mtrAngB) + 0.5*sin(mtrAngA))
#define DELTA_X(mtrAngA, mtrAngB) (-0.5*cos(mtrAngB) - 0.5*cos(mtrAngA))
#define LEG_LENGTH(mtrAngA, mtrAngB) sqrt(DELTA_Y(mtrAngA, mtrAngB)*DELTA_Y(mtrAngA, mtrAngB) + DELTA_X(mtrAngA, mtrAngB)*DELTA_X(mtrAngA, mtrAngB))
#define LEG_ANGLE(mtrAngA, mtrAngB) ((0.0*PI) - (atan2(DELTA_Y(mtrAngA, mtrAngB), DELTA_X(mtrAngA, mtrAngB))))

#endif // DRL_MATH_H

