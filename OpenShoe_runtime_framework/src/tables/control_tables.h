

/** \file
	\brief Header file for all tabulated information for the control and communication.
	
	\details This file contains declarations of arrays containing tabulated
	information about external system states, processing functions, and
	commands. It also contains struct typedefs of structs containing such
	information for individual states, functions, and commands, together
	with ID macros for the same.
	Delcarations of initialization functions for the arrays are also found.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/ 

/**
	\ingroup openshoe_runtime_framework
	
	\defgroup control_tables Tabulated information	
	\brief This group contains tabulated information for the control and communication.
	@{
*/


#ifndef CONTROL_TABLES_H_
#define CONTROL_TABLES_H_


#include "compiler.h"
#include "nav_types.h"

/// Definition structure of commands
typedef const struct {
	uint8_t header;
	void (*cmd_response)(uint8_t**);
	uint8_t nrb_payload;
	uint8_t nr_fields;
	uint8_t field_widths[];
} command_structure;

/// Information struct for processing functions
typedef const struct {
	uint8_t id;
	void (*func_p)(void);
	int max_proc_time; 
} proc_func_info;

/// State data type information struct
typedef const struct {
	uint8_t id;
	void* state_p;
	int state_size;
} state_t_info;


///  \name Processing functions IDs
///  Macros for processing functions IDs
//@{
#define UPDATE_BUFFER 0x04
#define INITIAL_ALIGNMENT 0x05
#define MECHANIZATION 0x06
#define TIME_UPDATE 0x07
#define ZUPT_DETECTOR 0x08
#define ZUPT_UPDATE 0x09
#define GYRO_CALIBRATION 0x10
#define ACCELEROMETER_CALIBRATION 0x11
//@}

///  \name External state IDs
///  Macros for external state IDs
//@{
// Maximum value of state ID (255)
#define SID_LIMIT 0xFF
// State IDs
// IMU measurements
#define SPECIFIC_FORCE_SID 0x01
#define ANGULAR_RATE_SID 0x02
#define IMU_TEMPERATURS_SID 0x03
#define IMU_SUPPLY_VOLTAGE_SID 0x04
// Filtering states
#define POSITION_SID 0x11
#define VELOCITY_SID 0x12
#define QUATERNION_SID 0x13
#define ZUPT_SID 0x14
// System states
#define INTERRUPT_COUNTER_SID 0x21
// "Other" states
#define ACCELEROMETER_BIASES_SID 0x35
//@}

///  \name Command IDs
///  Macros for command IDs
//@{
#define ONLY_ACK 0x01
#define MCU_ID 0x02
#define OUTPUT_STATE 0x20
#define OUTPUT_ALL_OFF 0x21
#define OUTPUT_ONOFF_INERT 0x22
#define OUTPUT_POSITION_PLUS_ZUPT 0x23
#define OUTPUT_NAVIGATIONAL_STATES 0x24
#define PROCESSING_FUNCTION_ONOFF 0x30
#define RESET_ZUPT_AIDED_INS 0x10
#define GYRO_CALIBRATION_INIT 0x11
#define ACC_CALIBRATION_INIT 0x12
#define SET_LOWPASS_FILTER_IMU 0x13
#define ADD_SYNC_OUTPUT 0x25
#define SYNC_OUTPUT 0x26
//@}

// Global variables used to access command information
extern uint8_t command_header_table[32];
extern command_structure* command_info_array[256];
void commands_init(void);

inline bool is_valid_header(uint8_t header){
	return command_header_table[header>>3] & (1<<(header & 7));}
	
inline command_structure* get_command_info(uint8_t header){
	return command_info_array[header];}


// Array containing the processing functions to run
extern proc_func_info* processing_functions_by_id[256];
void processing_functions_init(void);


// Global variables used to access information about states
extern state_t_info* state_info_access_by_id[SID_LIMIT];
void system_states_init(void);


#endif /* CONTROL_TABLES_H_ */

//@}