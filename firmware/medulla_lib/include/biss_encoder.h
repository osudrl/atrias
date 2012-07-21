#ifndef BISS_ENCODER_H
#define BISS_ENCODER_H

/** @file
 *  @brief This driver implements communication between the xMega and a Biss C encoder.
 *
 *  This driver handles all the interface and data management reuqired for using
 *  a Biss-C encoder. To follow the Biss-C spec the driver must continually
 *  clock the CLK pin for an arbitrary amount of time until the encoder finishes
 *  it's measurement and sends an Ack signal. This means that every call to
 *  biss_encoder_start_reading() takes approximately 18 uS to return. 
 *
 *  This driver can also generate a timestamp for the exact time when the
 *  encoder position was sampled. A pointer to a timer counter must be passed
 *  in, when the encoder is sampled, the value of this counter is also stored.
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "spi.h"

/** @brief This struct stores the confiuration of an encoder.
 */
typedef struct {
	spi_port_t spi_port; /**< SPI port used to communicate with the encoder */
	TC0_t *timestamp_timer; /**< Timer register to read the timestamp from */
	uint32_t *data_pointer; /**< Pointer to where the data is being stored */
	uint16_t *timestamp_pointer; /**< Pointer to where the timestamp is to be stored */
	uint8_t input_buffer[5]; /**< An array of 5 bytes where the data is clocked into */
	uint16_t cnt_per_us; /**< The number of timestep counts per milisecond */
} biss_encoder_t;

/** @brief Alias the SPI interrupt macro
 *
 *  This Alias is simply to make the user's program easier to understad. Instead
 *  of having to use SPI_USES_PORT for the BISS encoder you can instead use this
 *  macro. This macro adds no extra functionality to SPI_USES_PORT(SPI_PORT)
 *  
 *  @param SPI_PORT SPI_t struct for which to define the interrupt.
 */ 
#define BISS_ENCODER_USES_PORT SPI_USES_PORT

/** @brief Creates and initilizes a Biss-C encoder struct
 *
 *  This function sets up a new Biss-C encoder. The timestamp_timer pointer 
 *  is defined as void pointers so that either a TC0_t or TC1_t pointer can be
 *  passed into this function. Do not pass anything other than a TC0_t* or TC1_t*
 *  into this parameter.
 *
 *  The cnt_per_us parmeter should be the number of clock ticks the timestamp_timer
 *  increments by in one microsecond. This value is used to generate accurate
 *  timestamps.
 *
 *  @param spi_port Pointer to the IO port register the encoder is conencted to.
 *  @param spi_register Pointer to the SPI register that the encoder is
 *  connected to.
 *  @param timestamp_timer Pointer to the register of the timer used to generate
 *  the timestamps.
 *  @param cnt_per_us Number of timestamp_timer ticks in one microsecond.
 *  @param data_pointer Pointer to a location to store the encoder posision.
 *  @param timestamp_pointer Pointer to the location to write the timestamp.
 *  @return Retuns the newly configured biss_encoder_t struct
 */
biss_encoder_t biss_encoder_init(PORT_t *spi_port, SPI_t *spi_register, void *timestamp_timer, uint16_t cnt_per_us, uint32_t *data_pointer, uint16_t *timestamp_pointer);

/** @brief Starts an encoder read cycle
 *
 *  This function starts reading the position from the encoder. This function
 *  manually sends out all the clock pulses needed before data actually starts
 *  being received. Because this means the funcion has to wait for the Ack
 *  signal from the encoder, this function usually takes ~18uS to complete, and
 *  could take as long as 20uS.
 *
 *  @param encoder Pointer to the biss encoder struct for the encoder to read
 *  @return 0 - Read sucessfully started
 *  @return -1 - Read is already underway, new read has not been started.
 *  @return -2 - The encoder is reporting that it is not ready to be read.
 */
int biss_encoder_start_reading(biss_encoder_t *encoder);

/** @brief Processes the data received from the encoder
 *
 *  This function takes the data that was just read from the encoder, processes
 *  it, and places it and the timestamp into the appropreate locations in
 *  memory. This funciton also returns the status byte returned by the encoder.
 *  This function needs to get called once after a read completes. If not, the
 *  data in the output pointers will not be valid.
 *
 *  @param encoder Pointer to the biss encoder struct for the encoder to read
 *  @return status byte returned by encoder.
 */
uint8_t biss_encoder_process_data(biss_encoder_t *encoder);

/** @brief Checks if an encoder read is in progress
 *
 *  This function checks if the the encoder is currently being read from.
 *
 *  @param encoder Pointer to the biss encoder struct for the encoder to read
 *  @return true - The read has completed and the interface is idle.
 *  @return false - The encoder posision is currently being read.
 */
bool biss_encoder_read_complete(biss_encoder_t *encoder);

/** @brief Parses the status byte and returns if the read contains valid data.
 *
 *  Whenever the postion is read from the encoder, the encoder retuns a status
 *  byte which alerts us to any error. If the encoder was unable to read the
 *  encoder strip, it will send invalid data and set the error bit in the status
 *  byte. This function returns true if the data in the input buffer is valid
 *  and false if it is not. This function can be called before biss_process_data
 *  to decide if the input buffer should be parsed.
 *
 *  @param encoder Pointer to the biss encoder struct for the encoder to read
 *  @return true - Data in input buffer is valid
 *  @return false - Data in input buffer is bad and should not be used.
 */
bool biss_encoder_data_valid(biss_encoder_t *encoder);

#endif //BISS_ENCODER_H
