#ifndef ELABSTEST_H
#define ELABSTEST_H

#include <ecrt.h>
#include <ec_rtdm.h>
#include <rtdm/rtdm.h>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <time.h>
#include <errno.h>

#include <robot_invariant_defs.h>

#define VENDOR_ID MEDULLA_VENDOR_ID
#define PRODUCT_CODE0 MEDULLA_TEST_PRODUCT_CODE
#define PRODUCT_CODE1 MEDULLA_LEG_PRODUCT_CODE
#define RT_DEV_FILE "ec_rtdm0"
#define LOOP_PERIOD_NS 1000000
#define LOOP_OFFSET_NS  100000
#define EC_NEWTIMEVAL2NANO(TV) \
    (((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)

/* Master 0, Slave 0, "ATRIAS 2.1 EtherCAT Test (Medulla 1.5)"
 * Vendor ID:       0x0000060f
 * Product code:    0x00000004
 * Revision number: 0x00000003
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

/* Master 0, Slave 1, "ATRIAS 2.1 Leg (Medulla 1.5)"
 * Vendor ID:       0x0000060f
 * Product code:    0x00000001
 * Revision number: 0x00000005
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Command */
    {0x0006, 0x01, 16}, /* Counter */
    {0x0004, 0x01, 32}, /* Motor Current */
    {0x0005, 0x02, 8}, /* ID */
    {0x0005, 0x03, 8}, /* State */
    {0x0005, 0x04, 8}, /* Counter */
    {0x0005, 0x05, 8}, /* Error Flags */
    {0x0005, 0x06, 8}, /* Limit Switch */
    {0x0006, 0x02, 16}, /* Toe Sensor */
    {0x0007, 0x01, 32}, /* Motor Encoder */
    {0x0006, 0x03, 16}, /* Motor Encoder Timestamp */
    {0x0007, 0x02, 32}, /* Leg Encoder */
    {0x0006, 0x04, 16}, /* Leg Encoder Timestamp */
    {0x0006, 0x05, 16}, /* Incremental Encoder */
    {0x0006, 0x06, 16}, /* Incremental Encoder Timestamp */
    {0x0006, 0x07, 16}, /* Motor Voltage */
    {0x0006, 0x08, 16}, /* Logic Voltage */
    {0x0006, 0x09, 16}, /* Thermistor 0 */
    {0x0006, 0x0a, 16}, /* Thermistor 1 */
    {0x0006, 0x0b, 16}, /* Thermistor 2 */
    {0x0006, 0x0c, 16}, /* Thermistor 3 */
    {0x0006, 0x0d, 16}, /* Thermistor 4 */
    {0x0006, 0x11, 16}, /* Thermistor 5 */
    {0x0003, 0x01, 16}, /* Amp 1 Measured Current */
    {0x0003, 0x02, 16}, /* Amp 2 Measured Current */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 3, slave_1_pdo_entries + 0}, /* uControllerInput */
    {0x1a00, 6, slave_1_pdo_entries + 3}, /* uController Status */
    {0x1a01, 2, slave_1_pdo_entries + 9}, /* Motor Encoder */
    {0x1a02, 2, slave_1_pdo_entries + 11}, /* Leg Encoder */
    {0x1a03, 2, slave_1_pdo_entries + 13}, /* Incremental Encoder */
    {0x1a04, 2, slave_1_pdo_entries + 15}, /* Power */
    {0x1a05, 6, slave_1_pdo_entries + 17}, /* Thermistors */
    {0x1a06, 2, slave_1_pdo_entries + 23}, /* Measured Current */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 7, slave_1_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


using namespace RTT;

class ELabsTest : public TaskContext {
	ec_master_t* ec_master;
	ec_domain_t* domain;
	int rt_fd;
	int counter;
	CstructMstrAttach MstrAttach;
	ec_slave_config_t* sc0;
	ec_slave_config_t* sc1;
	public:
		ELabsTest(std::string name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

#endif // ELABSTEST_H
