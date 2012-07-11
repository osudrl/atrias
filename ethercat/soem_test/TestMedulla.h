#ifndef TESTMEDULLA_H
#define TESTMEDULLA_H

/** @file
  * @brief Provides info specific to the Test Medulla
  */

extern "C" {
#include <ethercattype.h>
#include <ethercatmain.h>
}
#include <stdint.h>

#define TEST_PRODUCT_CODE 1

//! @brief Contains pointers to the data received from and transmitted to this type of Medulla.
class TestMedulla {
	public:
		/** @brief Does SOEM's slave-specific init.
		  * @param slave A pointer to this slave.
		  */
		TestMedulla(ec_slavet* slave);
		
		// Stuff sent to the Test medullas
		uint8_t*  command;
		uint16_t* current;
		
		// Stuff received from the Test Medullas
		uint16_t* timestamp;
		uint32_t* encoder;
};

#endif // TESTMEDULLA_H
