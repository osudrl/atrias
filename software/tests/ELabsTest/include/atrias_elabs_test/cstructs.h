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

