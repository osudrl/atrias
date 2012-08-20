// Devin Koepl

#ifndef DRL_MATH_H
#define DRL_MATH_H


#define PI 3.14159265

#define ABS(x)	(((x) < 0) ? -(x) : (x))
#define CLAMP(x, min, max) ( ((x) < min) ? (min):( ((x) > max) ? (max):(x) ) )
#define SIGN(x) (((x) < 0) ? (-1) : (1))

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#define DISCRETIZE(VAL, VAL_MIN, VAL_MAX, COUNT_MIN, COUNT_MAX) \
    ((uint32_t)((VAL - VAL_MIN) / (VAL_MAX - VAL_MIN) * ((double)COUNT_MAX - (double)COUNT_MIN) + (double)COUNT_MIN))

#define UNDISCRETIZE(COUNT, VAL_MIN, VAL_MAX, COUNT_MIN, COUNT_MAX) \
    (((float)COUNT - (float)COUNT_MIN) * (VAL_MAX - VAL_MIN) / ((float)COUNT_MAX - (float)COUNT_MIN) + VAL_MIN)

#define DISCRETIZE_LOCATION(COUNT, COUNT_KNOWN, ANGLE_KNOWN, COUNTS_PER_REV, GEAR_RATIO) \
    (1./(float)(GEAR_RATIO) * 2. * PI / (float)COUNTS_PER_REV * ((float)COUNT - (float)COUNT_KNOWN) + (float)ANGLE_KNOWN)

/*
 * These are used by a few controllers to calculate leg lengths and angles from
 * the motor angles. I'm putting them here in case they ever need to be
 * changed.
 */
#define LEG_LENGTH(mtrAngA, mtrAngB) (cos(((mtrAngA) - (mtrAngB)) / 2.))
#define LEG_ANGLE(mtrAngA, mtrAngB) (((mtrAngA) + (mtrAngB)) / 2.)

/*#define DELTA_Y(mtrAngA, mtrAngB) (0.5*sin(mtrAngB) + 0.5*sin(mtrAngA))
#define DELTA_X(mtrAngA, mtrAngB) (-0.5*cos(mtrAngB) - 0.5*cos(mtrAngA))
#define LEG_LENGTH(mtrAngA, mtrAngB) sqrt(DELTA_Y(mtrAngA, mtrAngB)*DELTA_Y(mtrAngA, mtrAngB) + DELTA_X(mtrAngA, mtrAngB)*DELTA_X(mtrAngA, mtrAngB))
#define LEG_ANGLE(mtrAngA, mtrAngB) ((0.0*PI) - (atan2(DELTA_Y(mtrAngA, mtrAngB), DELTA_X(mtrAngA, mtrAngB))))*/

#endif // DRL_MATH_H

