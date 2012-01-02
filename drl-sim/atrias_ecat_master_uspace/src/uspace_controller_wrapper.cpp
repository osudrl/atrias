// Devin Koepl

#include "uspace_controller_wrapper.h"

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    //rt_sem_wait(&master_sem);
    ecrt_master_state(master, &ms);
    //rt_sem_signal(&master_sem);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/*****************************************************************************/

// This needs to be run at 1 kHz.
void cyclic_task()
{
    int i;

    // receive process data
    //rt_sem_wait(&master_sem);
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    //rt_sem_signal(&master_sem);

    //control_wrapper_state_machine( uc_in, uc_out );

    // send process data
    //rt_sem_wait(&master_sem);
    ecrt_domain_queue(domain1);
    //rt_sem_signal(&master_sem);
    ecrt_master_send(master);

    //rt_task_wait_period();
}

/*****************************************************************************/

//void request_lock_callback(void *cb_data)
//{
//    rt_sem_wait(&master_sem);
//}

/*****************************************************************************/

//void release_lock_callback(void *cb_data)
//{
//    rt_sem_signal(&master_sem);
//}

/*****************************************************************************/

int main(int argc, char **argv)
{
    //ec_slave_config_t *sc;
    //struct sigaction sa;
    //struct itimerval tv;
    
    printf("Requesting master 0...\n");
    if (!(master = ecrt_request_master(0))) {
        fprintf(stderr, "Requesting master 0 failed!\n");
        return -1;
    }

    printf("Registering domain...\n");
    if (!(domain1 = ecrt_master_create_domain(master))) {
        fprintf(stderr, "Domain creation failed!\n");
        return -1;
    }

    // Verify that four slaves are responding, i.e., we are attached to ATRIAS.
    check_master_state();
    if (master_state.slaves_responding != NUM_OF_MEDULLAS_ON_ROBOT) {
        fprintf(stderr, "%u Medullas detected.\n", master_state.slaves_responding);
        return -1;
    }

    printf("Getting slave configurations...\n");
    if (!(sc_medulla_boom = ecrt_master_slave_config(master, MEDULLA_BOOM_POS, VENDOR_ID, PRODUCT_CODE))) {
        fprintf(stderr, "Failed to get Medulla BOOM slave configuration.\n");
        return -1;
    }

    if (!(sc_medullaB = ecrt_master_slave_config(master, MEDULLA_B_POS, VENDOR_ID, PRODUCT_CODE))) {
        fprintf(stderr, "Failed to get Medulla B slave configuration.\n");
        return -1;
    }

    if (!(sc_medullaA = ecrt_master_slave_config(master, MEDULLA_A_POS, VENDOR_ID, PRODUCT_CODE))) {
        fprintf(stderr, "Failed to get Medulla A slave configuration.\n");
        return -1;
    }

    if (!(sc_medulla_hip = ecrt_master_slave_config(master, MEDULLA_HIP_POS, VENDOR_ID, PRODUCT_CODE))) {
        fprintf(stderr, "Failed to get Medulla HIP slave configuration.\n");
        return -1;
    }

#if CONFIGURE_PDOS
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_medulla_boom, EC_END, medulla_boom_sync)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_medullaB, EC_END, medullaB_sync)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_medullaA, EC_END, medullaA_sync)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_medulla_hip, EC_END, medulla_hip_sync)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
#endif // CONFIGURE_PDOS

    printf("Registering PDO entries...\n");
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master!\n");
        return -1;
    }

    // Get internal process data for domain.
    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }


    // Map I/O pointers to sync manager memory.
    uc_in[0] = (uControllerInput*) (domain1_pd + off_medullaA_rx);
    uc_in[1] = (uControllerInput*) (domain1_pd + off_medullaB_rx);
    //uc_in[2] = (uControllerInput*) (domain1_pd + off_medulla_hip_rx);
    //uc_in[0] = (uControllerInput*) (domain1_pd + off_medulla_hip_rx);

    uc_out[0] = (uControllerOutput*) (domain1_pd + off_medullaA_tx);
    uc_out[1] = (uControllerOutput*) (domain1_pd + off_medullaB_tx);
    //uc_out[2] = (uControllerOutput*) (domain1_pd + off_medulla_hip_tx);
    //uc_out[0] = (uControllerOutput*) (domain1_pd + off_medulla_hip_tx);


    //sa.sa_handler = signal_handler;
    //sigemptyset(&sa.sa_mask);
    //sa.sa_flags = 0;
    //if (sigaction(SIGALRM, &sa, 0)) {
    //    fprintf(stderr, "Failed to install signal handler!\n");
    //    return -1;
    //}

    //printf("Starting timer...\n");
    //tv.it_interval.tv_sec = 0;
    //tv.it_interval.tv_usec = 1000000 / FREQUENCY;
    //tv.it_value.tv_sec = 0;
    //tv.it_value.tv_usec = 1000;
    //if (setitimer(ITIMER_REAL, &tv, NULL)) {
    //    fprintf(stderr, "Failed to start timer: %s\n", strerror(errno));
    //    return 1;
    //}

    printf("Started.\n");
    while (1) {
        pause();

#if 0
        struct timeval t;
        gettimeofday(&t, NULL);
        printf("%u.%06u\n", t.tv_sec, t.tv_usec);
#endif

        while (1) {
            cyclic_task();
        }
    }

    return 0;
}

