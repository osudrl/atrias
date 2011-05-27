// Devin Koepl

#ifndef FUNCS_H_MEDULLA_SAFETY_TESTER
#define FUNCS_H_MEDULLA_SAFETY_TESTER

#define SHMNAM "USPACE_KERN_SHM"

typedef struct
{
	unsigned int				medullaA_biss_encoder;

	unsigned short int	medullaA_ssi_encoder;

	unsigned short int	timestep;

	unsigned char				status_byte;

	unsigned char				lock;
	unsigned char				fresh;
} UspaceKernShm;

#endif // FUNCS_H_MEDULLA_SAFETY_TESTER
