
/** \file
	\brief Declaration of external system states.
	
	\details This files contain extern declaration and definition of static state
	structs for all external states. External states mean that these states
	might be requested from the system. Consequently, if any states are to be
	output from the system, they should be added here.
	
	The file also contain an initialization function to fill up some arrays of
	states structs. This initialization function must be called before the
	arrays are used. This should eventually be replaced by some code generating
	script which explicitly declear the arrays.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

/**
	\addtogroup control_tables
	@{
*/

#include "control_tables.h"

///\cond
// IMU measurements
extern vec3 accelerations_in;
extern vec3 angular_rates_in;
extern vec3 imu_temperaturs;
extern precision imu_supply_voltage;
	
// Filtering states
extern vec3 position;
extern vec3 velocity;
extern quat_vec quaternions;
extern bool zupt;

// System states
extern uint32_t interrupt_counter;

// "Other" states
extern vec3 accelerometer_biases;
///\endcond

///  \name External state information
///  Structs containing information and pointers to the externally accessible system states.
//@{
static state_t_info specific_force_sti = {SPECIFIC_FORCE_SID, (void*) accelerations_in, sizeof(vec3)};
static state_t_info angular_rate_sti = {ANGULAR_RATE_SID, (void*) angular_rates_in, sizeof(vec3)};
static state_t_info imu_temperaturs_sti = {IMU_TEMPERATURS_SID, (void*) imu_temperaturs, sizeof(vec3)};
static state_t_info imu_supply_voltage_sti = {IMU_SUPPLY_VOLTAGE_SID, (void*) &imu_supply_voltage, sizeof(precision)};
static state_t_info position_sti = {POSITION_SID, (void*) position, sizeof(vec3)};
static state_t_info velocity_sti = {VELOCITY_SID, (void*) velocity, sizeof(vec3)};
static state_t_info quaternions_sti = {QUATERNION_SID, (void*) quaternions, sizeof(quat_vec)};
static state_t_info zupt_sti = {ZUPT_SID, (void*) &zupt, sizeof(bool)};
static state_t_info interrupt_counter_sti = {INTERRUPT_COUNTER_SID, (void*) &interrupt_counter, sizeof(uint32_t)};
	
static state_t_info accelerometer_biases_sti = {ACCELEROMETER_BIASES_SID, (void*) &accelerometer_biases, sizeof(vec3)};
//@}
	
// Array of state data type struct pointers
const static state_t_info* state_struct_array[] = {&interrupt_counter_sti,
												   &specific_force_sti,
												   &angular_rate_sti,
												   &imu_temperaturs_sti,
												   &imu_supply_voltage_sti,
												   &position_sti,
								 	               &velocity_sti,
												   &quaternions_sti,
												   &zupt_sti,
												   &accelerometer_biases_sti};


state_t_info* state_info_access_by_id[SID_LIMIT];

void system_states_init(void){
	for(int i = 0;i<(sizeof(state_struct_array)/sizeof(state_struct_array[0])); i++){
		state_info_access_by_id[state_struct_array[i]->id] = state_struct_array[i];}
}	


//@}