
/** \file
	\brief High level IMU interface header file.
	
	\details
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)	
*/ 

/**
	\ingroup openshoe_runtime_framework
	
	\defgroup imu_interface IMU interface	
	\brief This group contains the IMU interface functionalities.
	@{
*/

#ifndef IMU_SPI_INTERFACE_H_
#define IMU_SPI_INTERFACE_H_

#include "compiler.h"

//void imu_interupt_init(void);

/// Initialization routine for the IMU to MCU interface
void imu_interface_init(void);
// Routine for fast reading of vcc, acc, gyro, and temp from IMU
void imu_burst_read(void);
// Routine for setting number of filter taps in the IMU
void low_pass_filter_setting(uint8_t nr_filter_taps);

// Routine for reading only acc and gryo from IMU
//void imu_read_acc_and_gyro(void);



#endif /* IMU_SPI_INTERFACE_H_ */

//@}
