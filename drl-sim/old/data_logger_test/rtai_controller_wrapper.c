// Devin Koepl

#include "rtai_controller_wrapper.h"

/*****************************************************************************/

void run(long data) 
{
	int shm_index = 0;
	unsigned char cnt = 0;

	while ( true ) {
		t_last_cycle = get_cycles();

		shm[shm_index].data = cnt;
		shm[shm_index].fresh = true;

		shm_index++;
		shm_index = shm_index % SHM_ENTRIES;
		cnt++;

		rt_task_wait_period();
	}
}

/*****************************************************************************/

int __init init_mod(void) 
{
	int ret = -1;
	RTIME tick_period, requested_ticks, now;

	printk(KERN_INFO PFX "Starting...\n");

	shm = (DataEntry *)rtai_kmalloc(nam2num(SHMNAM), SHM_ENTRIES * sizeof(DataEntry));
	shm = (DataEntry *)rtai_kmalloc(nam2num(SHMNAM), SHM_ENTRIES * sizeof(DataEntry));
	shm = (DataEntry *)rtai_kmalloc(nam2num(SHMNAM), SHM_ENTRIES * sizeof(DataEntry));
	shm = (DataEntry *)rtai_kmalloc(nam2num(SHMNAM), SHM_ENTRIES * sizeof(DataEntry));
	if (shm == NULL)
		return -ENOMEM;
	memset(shm, 0, SHM_ENTRIES * sizeof(DataEntry));

	printk(KERN_INFO PFX "Starting cyclic sample thread...\n");
	requested_ticks = nano2count(TIMERTICKS);
	tick_period = start_rt_timer(requested_ticks);
	printk(KERN_INFO PFX "RT timer started with %i/%i ticks.\n",
		   (int) tick_period, (int) requested_ticks);

	if (rt_task_init(&task, run, 0, 2000, 0, 1, NULL)) {
		printk(KERN_ERR PFX "Failed to init RTAI task!\n");
		goto out_stop_timer;
	}

	now = rt_get_time();
	if (rt_task_make_periodic(&task, now + tick_period, tick_period)) {
		printk(KERN_ERR PFX "Failed to run RTAI task!\n");
		goto out_stop_task;
	}

	printk(KERN_INFO PFX "Initialized.\n");
	return 0;

 out_stop_task:
	rt_task_delete(&task);
 out_stop_timer:
	stop_rt_timer();
 out_return:
	rt_sem_delete(&master_sem);
	printk(KERN_ERR PFX "Failed to load. Aborting.\n");

	return ret;
}

/*****************************************************************************/

void __exit cleanup_mod(void)
{
	printk(KERN_INFO PFX "Stopping...\n");
	stop_rt_timer();
	rt_busy_sleep(10000000);
	rt_task_delete(&task);

	printk(KERN_INFO PFX "Unloading.\n");
	rtai_kfree(nam2num(SHMNAM));
	rtai_kfree(nam2num(SHMNAM));
	rtai_kfree(nam2num(SHMNAM));
	rtai_kfree(nam2num(SHMNAM));
	rtai_kfree(nam2num(SHMNAM));
	rtai_kfree(nam2num(SHMNAM));
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devin Koepl <devin.koepl@gmail.com>");
MODULE_DESCRIPTION("RTAI Datalogging Tester");

module_init(init_mod);
module_exit(cleanup_mod);

/*****************************************************************************/
