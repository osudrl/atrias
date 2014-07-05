#include <unistd.h>
#include <stdio.h>
#include <ecrt.h>
#include <signal.h>
#include <time.h>
#include <string.h>

/* The following should match the ID and product code (specific to type of
 * Medulla) in ESI config file. */
#define VENDOR_ID    0x060F
#define PRODUCT_CODE 0x0006

#define LOOP_PERIOD_NS 1000000

#define EC_NEWTIMEVAL2NANO(TV) \
    (((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)

volatile bool done = false;

/* The following output from 'ethercat cstruct' */

/* Master 0, Slave 0, "ATRIAS_2.1_IMU_(Medulla_1.5)"
 * Vendor ID:       0x0000060f
 * Product code:    0x00000006
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Command */
    {0x0006, 0x01, 16}, /* Counter */
    {0x0005, 0x02, 8}, /* ID */
    {0x0005, 0x03, 8}, /* State */
    {0x0005, 0x04, 8}, /* Counter */
    {0x0005, 0x05, 8}, /* Error Flags */
    {0x0008, 0x01, 32}, /* X */
    {0x0008, 0x02, 32}, /* Y */
    {0x0008, 0x03, 32}, /* Z */
    {0x0008, 0x04, 32}, /* X */
    {0x0008, 0x05, 32}, /* Y */
    {0x0008, 0x06, 32}, /* Z */
    {0x0005, 0x06, 8}, /* Status */
    {0x0005, 0x07, 8}, /* Sequence */
    {0x0003, 0x01, 16}, /* Temperature */
    {0x0007, 0x01, 32}, /* CRC */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1602, 2, slave_0_pdo_entries + 0}, /* uControllerInput */
    {0x1a0f, 4, slave_0_pdo_entries + 2}, /* uController Status */
    {0x1a10, 3, slave_0_pdo_entries + 6}, /* Rotation */
    {0x1a11, 3, slave_0_pdo_entries + 9}, /* Acceleration */
    {0x1a12, 4, slave_0_pdo_entries + 12}, /* Status */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

