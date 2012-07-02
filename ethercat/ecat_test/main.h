#include <unistd.h>
#include <stdio.h>
#include <ecrt.h>
#include <signal.h>
#include <time.h>

#define VENDOR_ID    0x060F
#define PRODUCT_CODE 0x0001

#define LOOP_PERIOD_NS 1000000

#define EC_NEWTIMEVAL2NANO(TV) \
    (((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)

volatile bool done = false;

/* Master 0, Slave 0, "ATRIAS 2.1 (Medulla 1.5)"
 * Vendor ID:       0x0000060f
 * Product code:    0x00000001
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Command */
    {0x0003, 0x02, 16}, /* Motor Current */
    {0x0006, 0x01, 16}, /* Timestep */
    {0x0007, 0x02, 32}, /* Encoder 0 */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 2, slave_0_pdo_entries + 0}, /* uControllerInput */
    {0x1a00, 2, slave_0_pdo_entries + 2}, /* uControllerOutput */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 1, "ATRIAS 2.1 (Medulla 1.5)"
 * Vendor ID:       0x0000060f
 * Product code:    0x00000001
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Command */
    {0x0003, 0x02, 16}, /* Motor Current */
    {0x0006, 0x01, 16}, /* Timestep */
    {0x0007, 0x02, 32}, /* Encoder 0 */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 2, slave_1_pdo_entries + 0}, /* uControllerInput */
    {0x1a00, 2, slave_1_pdo_entries + 2}, /* uControllerOutput */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
