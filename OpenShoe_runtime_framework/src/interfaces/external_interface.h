
/** \file
	\brief High level external user interface (USB) header file.
	
	\details 
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

/**
	\ingroup openshoe_runtime_framework
	
	\defgroup user_interface User (USB) interface	
	\brief This group contains high level functions for communication between user and system.
	@{
*/

#ifndef EXTERNAL_INTERFACE_H_
#define EXTERNAL_INTERFACE_H_

#include "compiler.h"

void com_interface_init(void);

// These are the two main functions used by the interface.
void transmit_data(void);
void receive_command(void);

// These functions are used by command response functions in commands.c
void set_state_output(uint8_t state_id, uint8_t divider);
void reset_output_counters(void);


// USB vbus callback function
///\cond
void vbus_event_callback(bool b_high);
///\endcond

#endif /* EXTERNAL_INTERFACE_H_ */

//@}