/**
 * \file
 *
 * \brief User board definition template
 *
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

/* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */

#include "compiler.h"

#ifdef __AVR32_ABI_COMPILER__ // Automatically defined when compiling for AVR32, not when assembling.
//#  include "led.h"
#endif  // __AVR32_ABI_COMPILER__


#ifdef AVR32_SCIF_101_H_INCLUDED
#define AVR32_SCIF_OSCCTRL0_STARTUP_2048_RCOSC  0x00000003
#define AVR32_SCIF_OSCCTRL0_STARTUP_16384_RCOSC 0x00000006
#define AVR32_SCIF_OSCCTRL32_STARTUP_8192_RCOSC 0x00000002
#endif



/*! \name Oscillator Definitions
 */
//! @{
	


//#define FOSC32          AVR32_SCIF_OSC32_FREQUENCY              //!< Osc32 frequency: Hz.
//#define OSC32_STARTUP   AVR32_SCIF_OSCCTRL32_STARTUP_8192_RCOSC //!< Osc32 startup time: RCOsc periods.

// Osc0 crystal is not mounted by default. Set the following definitions to the
// appropriate values if a custom Osc0 crystal is mounted on your STK600
//#define FOSC0           8000000                               //!< Osc0 frequency: Hz.
//#define OSC0_STARTUP    AVR32_SCIF_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.

// Osc1 crystal is not mounted by default. Set the following definitions to the
// appropriate values if a custom Osc1 crystal is mounted on your board.
// #define FOSC1           12000000                              //!< Osc1 frequency: Hz.
// #define OSC1_STARTUP    AVR32_SCIF_OSCCTRL1_STARTUP_2048_RCOSC  //!< Osc1 startup time: RCOsc periods.

//! @}

#define BOARD_OSC0_HZ           16000000
#define BOARD_OSC0_STARTUP_US   2000
#define BOARD_OSC0_IS_XTAL      true
#define BOARD_OSC32_HZ          32768
#define BOARD_OSC32_STARTUP_US  71000
#define BOARD_OSC32_IS_XTAL     true






/*! \name USB Definitions
 */
//! @{
//! Multiplexed pin used for USB_ID: AVR32_USBB_USB_ID_x_x.
//! To be selected according to the AVR32_USBB_USB_ID_x_x_PIN and
//! AVR32_USBB_USB_ID_x_x_FUNCTION definitions from <avr32/uc3cxxxx.h>.
/*#if (defined AVR32_USBB)
#  define USB_ID                             AVR32_USBB_ID_0_0
#else
#  define USB_ID                             AVR32_USBC_ID
#endif*/

//! Multiplexed pin used for USB_VBOF: AVR32_USBB_USB_VBOF_x_x.
//! To be selected according to the AVR32_USBB_USB_VBOF_x_x_PIN and
//! AVR32_USBB_USB_VBOF_x_x_FUNCTION definitions from <avr32/uc3cxxxx.h>.
/*#if (defined AVR32_USBB)
#  define USB_VBOF                           AVR32_USBB_VBOF_0_0
#else
#  define USB_VBOF                           AVR32_USBC_VBOF
#endif*/

//! Active level of the USB_VBOF output pin.
//#  define USB_VBOF_ACTIVE_LEVEL       LOW

//! USB overcurrent detection pin.
//#  define USB_OVERCURRENT_DETECT_PIN  AVR32_PIN_PB7

//! @}


/*! \name GPIO and SPI Connections of the SD/MMC Connector
 */
//! @{
/*#define SD_MMC_CARD_DETECT_PIN      AVR32_PIN_PA28
#define SD_MMC_WRITE_PROTECT_PIN    AVR32_PIN_PD30
#define SD_MMC_SPI                  (&AVR32_SPI1)
#define SD_MMC_SPI_NPCS             3
#define SD_MMC_SPI_SCK_PIN          AVR32_SPI1_SCK_0_1_PIN
#define SD_MMC_SPI_SCK_FUNCTION     AVR32_SPI1_SCK_0_1_FUNCTION
#define SD_MMC_SPI_MISO_PIN         AVR32_SPI1_MISO_0_1_PIN
#define SD_MMC_SPI_MISO_FUNCTION    AVR32_SPI1_MISO_0_1_FUNCTION
#define SD_MMC_SPI_MOSI_PIN         AVR32_SPI1_MOSI_0_1_PIN
#define SD_MMC_SPI_MOSI_FUNCTION    AVR32_SPI1_MOSI_0_1_FUNCTION
#define SD_MMC_SPI_NPCS_PIN         AVR32_SPI1_NPCS_3_2_PIN
#define SD_MMC_SPI_NPCS_FUNCTION    AVR32_SPI1_NPCS_3_2_FUNCTION*/
//! @}






/*! \name USART connection to the UC3B board controller
 */
//! @{
/*#define USART                        (&AVR32_USART2)
#define USART_RXD_PIN                AVR32_USART2_RXD_0_1_PIN
#define USART_RXD_FUNCTION           AVR32_USART2_RXD_0_1_FUNCTION
#define USART_TXD_PIN                AVR32_USART2_TXD_0_1_PIN
#define USART_TXD_FUNCTION           AVR32_USART2_TXD_0_1_FUNCTION
#define USART_IRQ                    AVR32_USART2_IRQ
#define USART_IRQ_GROUP              AVR32_USART2_IRQ_GROUP
#define USART_SYSCLK                 SYSCLK_USART2*/
//! @}



#endif // USER_BOARD_H
