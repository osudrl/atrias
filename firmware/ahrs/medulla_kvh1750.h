#ifndef MEDULLA_KVH1750_H
#define MEDULLA_KVH1750_H

#include <medulla.h>

void setup_kvh(void);
void read_kvh(float gyr[3], float acc[3]);
void print_imu();
void print_dcm(float dcm[3][3]);

#endif /* MEDULLA_KVH1750_H */

