#include <unistd.h>
#include <stdio.h>
#include <ecrt.h>
#include <signal.h>
#include <time.h>
#include <string.h>

#define VENDOR_ID    0x060F
#define PRODUCT_CODE 0x0006

#define LOOP_PERIOD_NS 1000000

#define EC_NEWTIMEVAL2NANO(TV) \
    (((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)

volatile bool done = false;
/* Master 0, Slave 0, "ATRIAS_2.1_IMU_(Medulla_1.5)"
 *  * Vendor ID:       0x0000060f
 *   * Product code:    0x00000006
 *    * Revision number: 0x00000001
 *     */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 1, 8}, /* Command */
    {0x0006, 2, 16}, /* Counter */
    {0x0005, 3, 8}, /* ID */
    {0x0005, 4, 8}, /* State */
    {0x0005, 5, 8}, /* Counter */
    {0x0005, 6, 8}, /* Error Flags */
    {0x0007, 7, 32}, /* X */
    {0x0007, 8, 32}, /* Y */
    {0x0007, 9, 32}, /* Z */
    {0x0007, 10, 32}, /* X */
    {0x0007, 11, 32}, /* Y */
    {0x0007, 12, 32}, /* Z */
    {0x0005, 13, 8}, /* Status */
    {0x0005, 14, 8}, /* Sequence */
    {0x0003, 15, 16}, /* Temperature */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1602, 2, slave_0_pdo_entries + 0}, /* uControllerInput */
    {0x1a0f, 4, slave_0_pdo_entries + 2}, /* uController Status */
    {0x1a10, 3, slave_0_pdo_entries + 6}, /* Rotation */
    {0x1a11, 3, slave_0_pdo_entries + 9}, /* Acceleration */
    {0x1a12, 3, slave_0_pdo_entries + 12}, /* Status */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

