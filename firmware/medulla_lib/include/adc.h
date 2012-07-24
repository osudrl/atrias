#ifndef ADC_H
#define ADC_H

/** @file
 *  @brief Provides a asyncronous interface to the xMega internal ADCs
 *  
 *  This driver provides an interrupt driven ADC interface which allows the user
 *  to start the ADC read cycle and all the ADCs will be read sequentially. The
 *  values from the ADC are stored in a temporary buffer. Calling
 *  adc_read_complete() will check the ADC has finished reading the inputs. If a
 *  read completes the ADC values will get copied into the given pointers.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>


/** @brief This struct stores the ADC configuration
 */
typedef struct {
	ADC_t *adc_port; /**< Pointer to the ADC registers this struct uses */
	volatile uint8_t pin_mask; /**< Bitmask for which ADC pins to read into the user's pointers */
	uint16_t* adc_output_pointers[8]; /**< Pointers into the user's memory where to place the ADC result */
} adc_port_t;

uint16_t _adc_buffer_ADCA[8], /**< Internal buffer to store conversion result from port A until it's read into user memory. */
         _adc_buffer_ADCB[8]; /**< Internal buffer to store conversion result from port B until it's read into user memory. */

/** @brief Macro to define ISR for an ADC port
 *
 *  This macro insersts an interrupt service function to be used with this
 *  driver. It handles reading the ADC results into the hardware buffers and
 *  switching the channel muxes to pins 4-7. This macro should be called at the
 *  top of the using program, to insert the ISR into the program.
 *
 *  @param ADC ADC_t port for the ADC to define the ISR for.
 **/
#define ADC_USES_PORT(ADC) \
ISR(ADC##_CH3_vect) { \
	if (_adc_buffer_##ADC[0] & 0x8000) { \
		/* We just read pins 0-3, so copy all the data into the port buffers */ \
		_adc_buffer_##ADC[0] = ADC.CH0.RES; \
		_adc_buffer_##ADC[1] = ADC.CH1.RES; \
		_adc_buffer_##ADC[2] = ADC.CH2.RES; \
		_adc_buffer_##ADC[3] = ADC.CH3.RES; \
		\
		/* Set up the second set of channels */ \
		ADC.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc; \
		ADC.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc; \
		ADC.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc; \
		ADC.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN7_gc; \
		/** Start the converter cycle for the new pins*/\
		ADC.CTRLA |= ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm | ADC_CH3START_bm; \
	} \
	else { \
		/* We just finished reading pins 4-7 so read that data into the port buffer */ \
		_adc_buffer_##ADC[4] = ADC.CH0.RES; \
		_adc_buffer_##ADC[5] = ADC.CH1.RES; \
		_adc_buffer_##ADC[6] = ADC.CH2.RES; \
		_adc_buffer_##ADC[7] = ADC.CH3.RES; \
	} \
} \

/** @brief Initilize a port of ADC inputs
 *  
 *  This function creates and returns a configured adc_port_t struct. It also
 *  configures the given analog to digital convertor to operate with this
 *  driver.
 *  
 *  @param adc Pointer to the ADC_t register for the port to configure.
 *  @return Returns a struct that can be used to reference the port.
 */
adc_port_t adc_init_port(ADC_t *adc);

/** @brief Initilizes a pin on an ADC port
 *
 *  This function configures a given pin on and ADC port. The pin_data pointer
 *  is the location to which the result of the ADC convertion will be written
 *  to. This function must be called for each pin that the ADC should read.
 *
 *  @param adc Pointer to an adc_porti_t struct to initilize the pin on.
 *  @param pin Zero indexed number of the pin to initilize
 *  @param pin_result Pointer to location to store result
 */
void adc_init_pin(adc_port_t *adc, uint8_t pin, uint16_t *pin_result);

/** @brief Starts the ADC read process.
 *
 *  This function kicks off the adc read of a port. It will start the ADC
 *  reading the first 4 pins of the port on the four channels, when these
 *  channels are complete, it will read the last four pins. adc_read_complete()
 *  can be called to get if the read has completed.
 *
 *  @param adc Pointer to the adc_port_t struct to start reading.
 */
void adc_start_read(adc_port_t *adc);

/** @brief Checks if the an ADC port has finished reading.
 *  
 *  This function checks if an ADC port conversion has completed. If it has
 *  completed then the data from the hardware buffer is coppied into the memory
 *  addresses provided for each pin.
 *
 *  @param adc Poitner to the adc_port_t struct to check read status.
 *  @return false - The ADC conversion is still in progress.
 *  @return true - The ADC conversion is finished, and the data has been copied into the given poitners.
 */
bool adc_read_complete(adc_port_t *adc);

#endif //ADC_H
