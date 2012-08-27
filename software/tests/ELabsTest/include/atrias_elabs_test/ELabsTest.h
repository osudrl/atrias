#ifndef ELABSTEST_H
#define ELABSTEST_H

#include <ecrt.h>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <time.h>

#include <robot_invariant_defs.h>

#define VENDOR_ID MEDULLA_VENDOR_ID
#define PRODUCT_CODE MEDULLA_TEST_PRODUCT_CODE

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

using namespace RTT;

class ELabsTest : public TaskContext {
	ec_master_t* ec_master;
	ec_domain_t* domain;
	public:
		ELabsTest(std::string name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void cleanupHook();
};

#endif // ELABSTEST_H
