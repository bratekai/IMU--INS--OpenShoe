

/** \file
	\brief Main function and interrupt control
	
	\details This file contains the main function and interrupt control.
	The main function control the execution of the program. The single
	interrupt routine only toggles a flag which the main function is polling.
	Each time the flag is toggled, the following will be executed in the main
	loop: 1) data is read from the IMU 2) functions in the process sequence
	are executed 3) commands are received from the user 4) data are
	transmitted back 5) and it is checked that a new interrupt did not arrived
	while the program was executing the main loop.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/ 

/**
  \defgroup openshoe_runtime_framework OpenShoe runtime framework
  \brief This module collect all software written for OpenShoe.
  
  \ingroup openshoe_runtime_framework
  @{
*/

/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */
#include <asf.h>

#include "process_sequence.h"
#include "external_interface.h"
#include "imu_interface.h"

// Interrupt counter (essentially a time stamp)
uint32_t interrupt_counter = 0;
// Global IMU interrupt (data) time-stamp variable
uint32_t imu_interrupt_ts;
// Variable that used to signal if an external interrupt occurs.
static volatile bool imu_interrupt_flag = false;
// Structure holding the configuration parameters of the EIC module.
static eic_options_t eic_options;

void imu_interupt_init(void){
	// EIC settings
	eic_options.eic_mode   = EIC_MODE_EDGE_TRIGGERED ;
	eic_options.eic_edge   = EIC_EDGE_RISING_EDGE ;
	eic_options.eic_async  = EIC_SYNCH_MODE;
	eic_options.eic_line   = IMU_INTERUPT_LINE1;
	eic_options.eic_filter = AVR32_EIC_FILTER_ON;
	
	Disable_global_interrupt();
	eic_init(&AVR32_EIC, &eic_options,IMU_INTERUPT_NB_LINES);
	eic_enable_line(&AVR32_EIC, eic_options.eic_line);
	eic_enable_interrupt_line(&AVR32_EIC, eic_options.eic_line);
	Enable_global_interrupt();
}

#if __GNUC__
__attribute__((__naked__))
#elif __ICCAVR32__
#pragma shadow_registers = full
#endif
// This handler is connected to the interrupt in "exception.S"
void eic_nmi_handler( void )
{
	// Save registers not saved upon NMI exception.
	__asm__ __volatile__ ("pushm   r0-r12, lr\n\t");
	
	eic_clear_interrupt_line(&AVR32_EIC, IMU_INTERUPT_LINE1);
	imu_interrupt_ts = Get_system_register(AVR32_COUNT);
	imu_interrupt_flag = true;
	
	// Significant amount of processing should not be done inside this routine
	// since the USB communication will be blocked for its duration.
	
	// Restore the registers and leaving the exception handler.
	__asm__ __volatile__ ("popm   r0-r12, lr\n\t" "rete");
}


/// Initialize hardware and communication interfaces
void system_init(void){
	irq_initialize_vectors();
	cpu_irq_enable();
	board_init();
	sysclk_init();
	com_interface_init();
	imu_interupt_init();
	imu_interface_init();
	// Any new initialization function of the system should be added here or
	// under any of the above initialization functions.
}

/// Wait for the interrupt flag to be set, toggle it, increase interrupt counter, and return.
void wait_for_interrupt(void){
	while(true){
		if(imu_interrupt_flag==true){
			imu_interrupt_flag=false;
			interrupt_counter++;
			return;
		}
	}	
}

/// Checks that the main loop has finished before next interrupt
void within_time_limit(void){
	if (imu_interrupt_flag!=false){
		// Todo: Set some error state
	}
}

int main (void) {
	
	// Initialize system
	system_init();
	
	// Main executing loop: Loops indefinitely
	while (true) {
		
		// Check if interrupt has occurred
		wait_for_interrupt();

		// Read data from IMU			
		imu_burst_read();

		// Execute all processing functions (filtering)
		run_process_sequence();

		// Check if any command has been sent and respond accordingly
		receive_command();
			
		// Transmit requested data to user
		transmit_data();
		
		// Ensure the loop was finished within time limit
		within_time_limit();
	}
}

//! @}