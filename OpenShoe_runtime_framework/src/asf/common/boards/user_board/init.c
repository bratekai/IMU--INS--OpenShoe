/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <board.h>
#include <conf_board.h>
#include "gpio.h"

void board_init(void)
{
	// GPIO pins and functions used for the IMU SPI interface
	static const gpio_map_t SPI_GPIO_MAP = {
		{AVR32_SPI0_SCK_PIN,  AVR32_SPI0_SCK_FUNCTION},
		{AVR32_SPI0_MISO_PIN, AVR32_SPI0_MISO_FUNCTION},
		{AVR32_SPI0_MOSI_PIN, AVR32_SPI0_MOSI_FUNCTION},
		{AVR32_SPI0_NPCS_0_PIN, AVR32_SPI0_NPCS_0_FUNCTION}				
	};

	// Assign GPIOs to SPI.
	gpio_enable_module(SPI_GPIO_MAP, sizeof(SPI_GPIO_MAP) / sizeof(SPI_GPIO_MAP[0]));
	
	// Map the interrupt line to appropriate GPIO pin and function
	gpio_enable_module_pin(IMU_INTERUPT_PIN,IMU_INTERUPT_FUNCTION);
	
}
