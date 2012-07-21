#ifndef SSI_ENCODER_H
#define SSI_ENCODER_H

/** @file
 *  @brief This driver implements communication between the xMega and an ssi encoder.
 *
 *  This driver handles all the interface and data management reuqired for using
 *  a SSI encoder. To follow the Biss-C spec the driver must continually
 *  clock the CLK pin for an arbitrary amount of time until the encoder finishes
 *  it's measurement and sends an Ack signal.
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
	uint8_t data_length; /**< Stores the number of bits to read from the encoder*/
	uint16_t *timestamp_pointer; /**< Pointer to where the timestamp is to be stored */
	uint8_t input_buffer[4]; /**< An array of 5 bytes where the data is clocked into */
} ssi_encoder_t;

/** @brief Alias the SPI interrupt macro
 *
 *  This Alias is simply to make the user's program easier to understad. Instead
 *  of having to use SPI_USES_PORT for the SSI encoder you can instead use this
 *  macro. This macro adds no extra functionality to SPI_USES_PORT(SPI_PORT)
 *  
 *  @param SPI_PORT SPI_t struct for which to define the interrupt.
 */ 
#define SSI_ENCODER_USES_PORT SPI_USES_PORT

/** @brief Creates and initilizes a SSI encoder struct
 *
 *  This function sets up a new SSI encoder. The timestamp_timer pointer is defined
 *  as void pointers so that either a TC0_t or TC1_t pointer can be passed into
 *  this function. Do not pass anything other than a TC0_t* or TC1_t* into this
 *  parameter.
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
 *  @param data_pointer Pointer to a location to store the encoder posision.
 *  @param data_length Number of bits of data to read from encoder
 *  @param timestamp_pointer Pointer to the location to write the timestamp.
 *  @return Retuns the newly configured biss_encoder_t struct
 */
ssi_encoder_t ssi_encoder_init(PORT_t *spi_port, SPI_t *spi_register, void *timestamp_timer, uint32_t *data_pointer, uint8_t data_length, uint16_t *timestamp_pointer);

/** @brief Starts an encoder read cycle
 *
 *  This function kicks off the asycronous read process to get the posision of
 *  the encoder. Because the spi hardware has to read in one byte at a time,
 *  this will manually read in any additional bits.
 *
 *  @param encoder Pointer to the biss encoder struct for the encoder to read
 *  @return 0 - Read sucessfully started
 *  @return -1 - Read is already underway, new read has not been started.
 *  @return -2 - The encoder is reporting that it is not ready to be read.
 */
int ssi_encoder_start_reading(ssi_encoder_t *encoder);

/** @brief Processes the data received from the encoder
 *
 *  This function takes the data that was just read from the encoder, processes
 *  it, and places it and the timestamp into the appropreate locations in
 *  memory. This function needs to get called once after a read completes. If not,
 *  the data in the output pointers will not be valid.
 *
 *  @param encoder Pointer to the ssi encoder struct for the encoder to read
 */
void ssi_encoder_process_data(ssi_encoder_t *encoder);

/** @brief Checks if an encoder read is in progress
 *
 *  This function checks if the the encoder is currently being read from.
 *
 *  @param encoder Pointer to the ssi encoder struct for the encoder to read
 *  @return true - The read has completed and the interface is idle.
 *  @return false - The encoder posision is currently being read.
 */
bool ssi_encoder_read_complete(ssi_encoder_t *encoder);

#endif //SSI_H
