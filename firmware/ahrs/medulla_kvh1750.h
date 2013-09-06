#ifndef MEDULLA_KVH1750_H
#define MEDULLA_KVH1750_H

#include <medulla.h>

// The DCM
float dcm_out[3][3];

void setup_kvh(void);
void read_kvh(float gyr[3], float acc[3]);

#endif /* MEDULLA_KVH1750_H */

