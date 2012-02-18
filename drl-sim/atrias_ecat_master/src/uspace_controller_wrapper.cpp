// Devin Koepl

#include <atrias_ecat_master/uspace_controller_wrapper.h>

/*****************************************************************************/

void check_domain1_state(void) {
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

void check_master_state(void) {
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

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
void cyclic_task(void) {
    //int i;

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state (optional)
    //check_domain1_state();

    // uc_in and uc_out are arrays of pointers to Medulla sync manager memory
    // addresses, defined in init_master().
    control_wrapper_state_machine( uc_in, uc_out );
    control_switcher_state_machine( &shm.controller_input, &shm.controller_output, &shm.controller_state, shm.controller_data );

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
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

// ============================================================================
// EtherLab user space example is a single executable with a main function.
// Here, we replace the main function with init_master() that will be called in
// the startupHook of an Orocos TaskContext.
// ============================================================================

//int main(int argc, char **argv) {
int init_master(void) {
    //ec_slave_config_t *sc;
    //struct sigaction sa;
    //struct itimerval tv;
    
    printf("[AEM] Requesting master 0...\n");
    if (!(master = ecrt_request_master(0))) {
        fprintf(stderr, "[AEM] Requesting master 0 failed!\n");
        return -1;
    }

    printf("[AEM] Registering domain...\n");
    if (!(domain1 = ecrt_master_create_domain(master))) {
        fprintf(stderr, "[AEM] Domain creation failed!\n");
        return -1;
    }

    // Check master state (i.e., get Medullas and stuff).
    check_master_state();

    // Verify that four slaves are responding, i.e., we are attached to ATRIAS.
    printf("[AEM] Verifying that %u Medullas are connected...\n", NUM_OF_MEDULLAS_ON_ROBOT);
    if (master_state.slaves_responding != NUM_OF_MEDULLAS_ON_ROBOT) {
        fprintf(stderr, "[AEM] %u Medullas detected.\n", master_state.slaves_responding);
        return -1;
    }
    else {
        atrias_connected = true;
    }

    printf("[AEM] Getting slave configurations...\n");
    if (!(sc_medulla_boom = ecrt_master_slave_config(master, MEDULLA_BOOM_POS, VENDOR_ID, PRODUCT_CODE))) {
        fprintf(stderr, "[AEM] Failed to get Medulla BOOM slave configuration.\n");
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

#ifdef CONFIGURE_PDOS
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
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs)) {
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
    // TODO: Replace the magic numbers below with #defines in some globals.h.
    printf("Mapping I/O pointers to sync manager memory.");
    uc_in[0] = (uControllerInput*) (domain1_pd + off_medullaA_rx);
    uc_in[1] = (uControllerInput*) (domain1_pd + off_medullaB_rx);
    uc_in[2] = (uControllerInput*) (domain1_pd + off_medulla_hip_rx);
    uc_in[3] = (uControllerInput*) (domain1_pd + off_medulla_boom_rx);

    uc_out[0] = (uControllerOutput*) (domain1_pd + off_medullaA_tx);
    uc_out[1] = (uControllerOutput*) (domain1_pd + off_medullaB_tx);
    uc_out[2] = (uControllerOutput*) (domain1_pd + off_medulla_hip_tx);
    uc_out[3] = (uControllerOutput*) (domain1_pd + off_medulla_boom_tx);

    printf("[UCW] EtherCAT master initialized.\n");

    return 0;
}

