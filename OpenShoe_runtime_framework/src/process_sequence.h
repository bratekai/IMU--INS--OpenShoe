
/** \file
	\brief Header file containing declarations for functions for manipulating and running process sequence.
	
	\details
		
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

/**
	\ingroup openshoe_runtime_framework
	
	\defgroup proc_sequence Processing sequence	
	\brief This groupe contains the processing sequence declarations and definitions.
	@{
*/

#ifndef PROCESS_SEQUENCE_H_
#define PROCESS_SEQUENCE_H_

#include "compiler.h"

/// Size of process sequence
#define PROCESS_SEQUENCE_SIZE 10
/// Typedefinition for functions in the process sequence
typedef void (*processing_function_p)(void);

void run_process_sequence(void);

void empty_process_sequence(void);

void store_and_empty_process_sequence(void);

void restore_process_sequence(void);

void set_last_process_sequence_element(processing_function_p element_value);

void set_elem_in_process_sequence(processing_function_p elem_value, uint8_t elem_nr);

#endif /* PROCESS_SEQUENCE_H_ */

//@}