#ifndef CPU_H
#define CPU_H

/** @file
 *  @brief Library for control several low level CPU systems.
 *
 *  This library is really a combination of several small drivers that control
 *  non-peripheral systems on the xMega. It includes functions for controlling
 *  the following systesms:
 *    - CPU clock
 *    - Real time clock
 *    - Interrupts
 *    - Software reset
 */

#include <avr/io.h>
#include <stdbool.h>

/** @brief Possible clock sources
 *
 *  This enum contains the possible clock sources to be used with
 *  cpu_set_clock_source(). The values in the enum can be used to directly set
 *  the clock source bits in the clock control register.
 */
typedef enum {
	cpu_2mhz_clock = ((uint16_t)OSC_RC2MEN_bm<<8) | CLK_SCLKSEL_RC2M_gc,
	cpu_32mhz_clock = ((uint16_t)OSC_RC32MEN_bm<<8) | CLK_SCLKSEL_RC32M_gc,
	cpu_32khz_clock = ((uint16_t)OSC_RC32KEN_bm<<8) | CLK_SCLKSEL_RC32K_gc,
} cpu_clock_source_t;

/** @brief xMega interrupt levels
 *  
 *  This struct is used to specify a particular interrupt level. This struct is
 *  used by cpu_configure_interrupt_level() to ensure type safety when the user
 *  spcifies a interrupt level to enable.
 */
typedef enum {
	cpu_interrupt_level_low = PMIC_LOLVLEN_bm,
	cpu_interrupt_level_medium = PMIC_MEDLVLEN_bm,
	cpu_interrupt_level_high = PMIC_HILVLEN_bm
} cpu_interrupt_level_t;

/** @brief Configures the system clock source
 *
 *  This function sets the source for the cpu and peripheral clocks. It will
 *  attempt to start oscillator input, if the source stabilizes then the system
 *  clock source will be changed and true will be reurned. If the clock cannot
 *  be started or does not stabilize, false will be returned
 *
 *  @param clk_source Source to attempt to switch the cpu clock to
 *  @return true  - The system clock was sucessfully changed
 *  @return false - Failed to switch clock sources (clock could not be started,
 *  or did not stabilize).
 */
bool cpu_set_clock_source(cpu_clock_source_t clk_source);

/** @brief Tries to confiture the internal real time clock.
 *
 *  This function tries to enable or disable the real time clock source in
 *  the xmega with the internal 32Khz oscillator. If the RTC is being enabled
 *  and the oscillator does not stablize, false is returned and the RTC is not
 *  started. If enabling the RTC is sucessful or a disable has been requested,
 *  this function returns true.
 *
 *  @param enabled If true an attempt will be made to start the RTC, if false,
 *  the RTC will be disabled.
 *  @return true - RTC sucessfully started or stopped
 *  @return false - RTC could not be started
 */
bool cpu_configure_rtc(bool enabled);

/** @brief Enable or disable an interrupt level
 *
 *  All interrupts on the xmega are associated with an interrupt level when they
 *  are enabled. To use an interrupt, that interrupt level has to be enabled
 *  before the interrutps will be generated. This function will enable or
 *  disable an interrupt level.
 *
 *  @note
 *  In addition to enabling the interrupt level, interrupts need globally
 *  enabled by calling sei(). If this is not done, no interrupt will be
 *  generated.
 *
 *  @param interrupt_level Interrupt level to configure
 *  @param enable Set to true to enable the interrupt level, set to false to
 *  disable.
 */
void cpu_configure_interrupt_level(cpu_interrupt_level_t interrupt_level, bool enable);

/** @brief Reset the xMega
 *  
 *  Calling this function will immediately cause the xMega to reset as if the
 *  reset button was pressed.
 */
void cpu_reset(void);

#endif // CPU_H
