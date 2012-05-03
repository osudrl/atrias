#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include <rtai_shm.h>
#include <rtai_nam2num.h>


#define SHMNAM "MYSHM"
#define SHM_ENTRIES 1000

/*****************************************************************************/

typedef struct
{
	unsigned char fresh;

	int data;
} DataEntry;

DataEntry *shm;

/*****************************************************************************/

static int end = 0;
static void endme (int dummy) { end = 1; }

int main (int argc, char* argv[])
{
	int shm_index = 0;

    DataEntry *shm;

    printf("nam2num(%s) = 0x%lx\n", SHMNAM, nam2num(SHMNAM));
    shm = (DataEntry *)rtai_malloc(nam2num(SHMNAM), 0);
    shm = (DataEntry *)rtai_malloc(nam2num(SHMNAM), 0);
    shm = (DataEntry *)rtai_malloc(nam2num(SHMNAM), 0);
    if (shm == NULL) 
	{
		printf("rtai_malloc() failed (maybe /dev/rtai_shm is missing)!\n");

		return -1;
    }

    signal(SIGINT, endme);

    while (!end) 
	{
		if ( shm[shm_index].fresh )
		{
			printf("%u: %u\n", shm_index, shm[shm_index].data);

			shm_index++;
			shm_index = shm_index % SHM_ENTRIES;
		}
    }

    rtai_free (nam2num(SHMNAM), shm);
    rtai_free (nam2num(SHMNAM), shm);
    rtai_free (nam2num(SHMNAM), shm);
    rtai_free (nam2num(SHMNAM), shm);

   return 0;
}

