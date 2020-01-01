/**
 * \file
 *
 * \brief User board definition template
 *
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

// IMU pins
#define IMU_INTERUPT_PIN		AVR32_EIC_EXTINT_0_1_PIN
#define IMU_INTERUPT_FUNCTION	AVR32_EIC_EXTINT_0_1_FUNCTION
#define IMU_INTERUPT_LINE1		EXT_NMI
#define IMU_INTERUPT_NB_LINES	1

// Board clock oscillator
#define BOARD_OSC0_HZ           16000000
#define BOARD_OSC0_STARTUP_US   2000
#define BOARD_OSC0_IS_XTAL      true

#endif // USER_BOARD_H
