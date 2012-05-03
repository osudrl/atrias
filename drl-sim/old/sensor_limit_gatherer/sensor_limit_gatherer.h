// Devin Koepl

#ifndef FUNCS_H_SENSOR_LIMIT_GATHERER
#define FUNCS_H_SENSOR_LIMIT_GATHERER

#define SHMNAM "USPACE_KERN_SHM"

typedef struct
{
	unsigned int				medullaA_biss_encoder;
	unsigned int				medullaB_biss_encoder;

	unsigned short int	medullaA_ssi_encoder;
	unsigned short int	medullaB_ssi_encoder;
	unsigned short int	medulla_hip_ssi_encoder;

	unsigned short int	boom_pan_enc;
	unsigned short int	boom_tilt_enc;
	unsigned short int	boom_roll_enc;

	unsigned short int	timestep;

	unsigned char				status_byte;

	unsigned char				lock;
	unsigned char				fresh;
} UspaceKernShm;

#endif // FUNCS_H_SENSOR_LIMIT_GATHERER
