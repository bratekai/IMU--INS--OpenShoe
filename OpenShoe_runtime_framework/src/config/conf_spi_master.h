/*! \file *********************************************************************
 *
 * \brief Spi Master configuration template file
 *
 * Copyright (C) 2010 Atmel Corporation. All rights reserved.
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
 
#ifndef CONF_SPI_MASTER_H_INCLUDED
#define CONF_SPI_MASTER_H_INCLUDED

#define CONFIG_SPI_MASTER_DELAY_BCS            0
#define CONFIG_SPI_MASTER_BITS_PER_TRANSFER    16
// Minimum delay between word transfers
#define CONFIG_SPI_MASTER_DELAY_BCT            1
#define CONFIG_SPI_MASTER_DELAY_BS             0
// 16-bit dummy data word for slave-to-master communication
#define CONFIG_SPI_MASTER_DUMMY                0xFFFF


#define SPI_IMU             (&AVR32_SPI0)
#define SPI_IMU_ID			AVR32_SPI0_NPCS_0_PIN
#define SPI_IMU_BAUDRATE    1000000



#endif /* CONF_SPI_MASTER_H_INCLUDED */
