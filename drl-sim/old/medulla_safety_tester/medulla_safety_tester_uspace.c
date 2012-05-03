// Devin Koepl

#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>

#include <rtai_shm.h>
#include <rtai_nam2num.h>

#include "medulla_safety_tester.h"

#include "/home/drl/drl-sim/drl-sim/atrias/include/atrias/ucontroller.h"
#include "/home/drl/drl-sim/drl-sim/drl_library/include/drl_library/discretize.h"

#define TRUE 1
#define FALSE 0

#define CHANGES_PER_WRITE 10

UspaceKernShm *kern_shm;

int main (int argc, char **argv)
{
	int changes = 0;
	int write_count = 0;

	//float segA;
	//float segB;
	//float tranA;

  kern_shm = (UspaceKernShm *)rtai_malloc(nam2num(SHMNAM), 0);
  kern_shm = (UspaceKernShm *)rtai_malloc(nam2num(SHMNAM), 0);
  kern_shm = (UspaceKernShm *)rtai_malloc(nam2num(SHMNAM), 0);
  kern_shm = (UspaceKernShm *)rtai_malloc(nam2num(SHMNAM), 0);
  if (kern_shm == NULL) 
	{
		perror("rtai_malloc() failed (maybe /dev/rtai_shm is missing)!");
  }

	FILE *fp = fopen("sensor_data_4med.dat", "w"); 
	if (!fp)
	{
		perror("Could not open file for writing.\n");
	}

	fprintf(fp, "A Seg, B Seg, A Tran, B Tran, Hip, Boom Pan, Boom Tilt, Boom Roll\n");

	while (write_count < 100000)
	//while (TRUE)
	{
		// First wait for the shm to become available.
		while (kern_shm->lock);
		kern_shm->lock 			 = TRUE;

		if (kern_shm->fresh)
		{
			kern_shm->fresh = FALSE;

			if (changes < 100)
			{
				/*segA		= UNDISCRETIZE(kern_shm->medullaA_biss_encoder,
					MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

				segB		= UNDISCRETIZE(kern_shm->medullaB_biss_encoder,
					MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

				printf("A: %u => %.3f\t\t\tB: %u => %.3f\n", kern_shm->medullaA_biss_encoder, segA,
					kern_shm->medullaB_biss_encoder, segB);*/

				/*tranA = UNDISCRETIZE(kern_shm->medullaA_ssi_encoder,
					MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT);

				printf("timestep: %u\tA: %u => %.3f\tsb: %02X\n", kern_shm->timestep, kern_shm->medullaA_ssi_encoder, tranA,
					kern_shm->status_byte);*/

				/*fprintf(fp, "%u, %u, %u, %u, %u, %u, %u, %u\n", kern_shm->medullaA_biss_encoder, kern_shm->medullaB_biss_encoder,
					kern_shm->medullaA_ssi_encoder, kern_shm->medullaB_ssi_encoder, kern_shm->medulla_hip_ssi_encoder,
					kern_shm->boom_pan_enc, kern_shm->boom_tilt_enc, kern_shm->boom_roll_enc);

				printf("%u, %u, %u, %u, %u, %u, %u, %u\n", kern_shm->medullaA_biss_encoder, kern_shm->medullaB_biss_encoder,
					kern_shm->medullaA_ssi_encoder, kern_shm->medullaB_ssi_encoder, kern_shm->medulla_hip_ssi_encoder,
					kern_shm->boom_pan_enc, kern_shm->boom_tilt_enc, kern_shm->boom_roll_enc);*/

				changes = CHANGES_PER_WRITE;
				write_count++;
			}
			else
			{
				changes --;
			}
		}

		kern_shm->lock			 = FALSE;
	}	

	fclose(fp);

  rtai_free (nam2num(SHMNAM), shm);
  rtai_free (nam2num(SHMNAM), shm);
  rtai_free (nam2num(SHMNAM), shm);
  rtai_free (nam2num(SHMNAM), shm);

	printf("Finished!\n");

	return 0;
}
