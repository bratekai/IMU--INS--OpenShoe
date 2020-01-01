/*! \file main.c
	\brief The main file for the OpenShoe navigation algorithm test framework. 
	
	\details This is the main file for the OpenShoe navigation algorithm test framework. The 
	algorithm test framework is used to compare the navigation algorithm implemented on the
	micro-controller with the navigation algorithm implemented in Matlab. The framework thus 
	runs two implementations of the zero-velocity aided inertial navigation system navigation  
	in parallel, one the micro controller and one in Matlab. 
	
	\authors John-Olof Nilsson, Isaac Skog
 	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
 */ 


// Includes
#include <asf.h>
#include "nav_types.h"
#include "nav_eq.h"


// Constants
extern precision latitude;						//Rough latitude of the system [degrees]. (Used to calculate the magnitude of the gravity vector)	
extern precision altitude;						//Rough altitude of the system [m]. (Used to calculate the magnitude of the gravity vector)
extern precision g;								//Gravity acceleration [m/s^2]
extern precision dt;							//Sampling period [s]



// Filter settings
extern precision sigma_acceleration;						//Accelerometer process noise STD [m/s^2]
extern precision sigma_gyroscope;							//Gyroscope process noise STD [rad/s]
extern vec3 sigma_velocity;								//ZUPT measurement noise STD [m/s]
extern vec3  sigma_initial_position;			      //STD of the initial position uncertainty [m]
extern vec3  sigma_initial_velocity;		         //STD of the initial velocity uncertainty [m/s]
extern vec3  sigma_initial_attitude;				//STD of the initial attitude uncertainty [rad]

//Navigation states 
extern vec3 position;					//Vector holding the position (North,East,Down) [m]
extern vec3 velocity;					//Vector holding the velocity (North,East,Down) [m/s]
extern quat_vec quaternions;			//Vector holding the attitude (quaternions)
extern mat3 Rb2t;						//Rotation matrix used as an "aiding" variable in the filter algorithm 
extern vec3 accelerations_out;				//Accelerations outputted from the IMU data buffer [m/s^2]	
extern vec3 angular_rates_out;			    //Angular rates outputted from the IMU data buffer [rad/s]
volatile vec3 accelerations_in;		//Accelerations read from the IMU [m/s^2]. These are written into the IMU data buffer.	
volatile vec3 angular_rates_in;		//Angular rates read from the IMU [rad/s]. These are written into the IMU data buffer.


//Filter state variables
extern mat9sym cov_vector;				//Vector holding the error covariance in the filter.
extern mat9by3 kalman_gain;			//Vector holding the Kalman filter gain
extern mat3sym Re;						//Innovation covariance matrix
extern mat3sym invRe;					//Inverse of the innovation covariance matrix

// Initial navigation state settings  
extern Bool initialize_flag;						//Flag controlling if the navigation algorithm should be initialized
extern uint8_t nr_of_inital_alignment_samples;		      //Number of samples used in the initial alignment
extern vec3 acceleration_mean;							//Mean acceleration vector, used in the initial alignment [m/s^2] 
extern precision initial_heading;							//Initial attitude (roll,pitch,heading) [rad]
extern vec3 initial_pos;							   //Initial position (North, East, Down) [m] 

// ZUPT detector settings
extern precision sigma_acc_det;							//Accelerometer noise STD used in the ZUPT detector [m/s^2]
extern precision sigma_gyro_det;						//Gyroscope noise STD used in the ZUPT detector [rad/s] 	
extern volatile uint8_t detector_Window_size;					//The data window size used in the ZUPT detector (OBS! Must be a odd number)
extern precision detector_threshold;					//Threshold used int ZUPT detector
extern Bool zupt;									//Flag that indicates if a zero-velocity update should be done.
extern precision Test_statistics;					//Variable holding the test statistics for the likelihood test	



//Accelerometer calibration variables
extern vec3 accelerometer_biases;								//Vector holding the accelerometer biases (x,y,z-axis) [m/s^2]; 		
extern precision acceleration_variance_threshold;					//Threshold used to check that accelerometers were stationary during the calibration [(m/s^2)^2]     
extern uint32_t nr_of_calibration_samples;						//Number of samples used at each orientation in the calibration procedure.
extern uint8_t nr_of_calibration_orientations;						//Number of orientations used in the accelerometer calibration procedure. OBS! Most be at least 3 and less than 13. 		     
extern Bool new_orientation_flag;								//Flag that indicates when the IMU should be place in a new orientation.
extern Bool acc_calibration_finished_flag;						//Flag that indicates that the accelerometer calibration was successful.


/// Vector used for storing the the number of clock cycles required to execute a function. 
volatile U32 clock_cycles_vec[6];

/// Variable that becomes non-zero if an error has occurred when executing a function.  
uint8_t error_signal=0;							  




/************************ FUNCTIONS DECLARATIONS  *******************************/

/*! \brief Delay function.
	
	 @param[in]		ms		The delay in milliseconds. 	 	 	 
*/ 
static void mdelay(unsigned int ms);

/*! \brief Function that reads the navigation algorithm settings sent from Matlab.  
*/ 
void get_settings_from_matlab(void);

/*! \brief Function that reads the inertial measurement unit data sent from Matlab.  
*/ 
void get_imu_data_from_matlab(void);

/*! \brief Function that writes the navigation state to Matlab.  
*/ 
void write_nav_data_to_matlab(void);

/*! \brief Function that writes the state covariance matrix (vector) to Matlab.  
*/ 
void write_cov_vector_data_to_matlab(void);

/*! \brief Function that writes the clock cycle counter vector to Matlab.  
*/ 
void write_clock_cycles_to_matlab(void);

/*! \brief Function that writes the state of the zero-velocity detector to Matlab.  
*/ 
void write_zupt_flag_to_matlab(void);


/*************************** MAIN **********************************/
int main (void)
{
	
	irq_initialize_vectors();
	cpu_irq_enable();
	
	// Initialize the board, system clk etc.
	board_init();
	sysclk_init();
	udc_start();
	
	// Variable used to store the clock register.  
	volatile U32 clock_cycles_ctr;

	
	//Delay needed in order for the USB port to start up.
	mdelay(3000);
	
	// Read the settings from Matlab.
	get_settings_from_matlab();
	
	// Newer ending story
	while (1) 
	{
	
	// Read new IMU data from Matlab and update the buffers
	get_imu_data_from_matlab();			 
	update_imu_data_buffers();	
		
		// If the initialization flag is true run the initialization, else
		// run the navigation algorithm. 
		if(initialize_flag)
		{	
			
			// Call the initialization function.	
			initialize_navigation_algorithm();
		
			//Send the result of the initialization to Matlab
			if(initialize_flag==false){
				write_nav_data_to_matlab();
				write_cov_vector_data_to_matlab();
				write_zupt_flag_to_matlab();
			}
				
		}	
		else
		{
			 
				 
		/*********** TIME UPDATE ***********/
	
		//Update the navigation states
		clock_cycles_ctr=Get_system_register(AVR32_COUNT);	
		strapdown_mechanisation_equations();
		clock_cycles_ctr=Get_system_register(AVR32_COUNT)-clock_cycles_ctr;
		clock_cycles_vec[0]=clock_cycles_ctr;
	
		
		//Calculate the a priori state covariance matrix
		clock_cycles_ctr=Get_system_register(AVR32_COUNT);	
		time_up_data();
		clock_cycles_ctr=Get_system_register(AVR32_COUNT)-clock_cycles_ctr;
		clock_cycles_vec[1]=clock_cycles_ctr;
	
		/**********************************/
	
	
	
		/*********** DETECTOR ************/
	 
		// Run the zero velocity detector
		clock_cycles_ctr=Get_system_register(AVR32_COUNT);	
		ZUPT_detector();
		clock_cycles_ctr=Get_system_register(AVR32_COUNT)-clock_cycles_ctr;
		clock_cycles_vec[2]=clock_cycles_ctr;
		/*********************************/
	
	
		/****** MEASUREMENT UPDATE *******/
		
		if(zupt)
		{
	
			// Calculate the Kalman filter gain
			clock_cycles_ctr=Get_system_register(AVR32_COUNT);	
			gain_matrix();
			clock_cycles_ctr=Get_system_register(AVR32_COUNT)-clock_cycles_ctr;
			clock_cycles_vec[3]=clock_cycles_ctr;
	
	
			// Correct the navigation states using the zero-velocity pseudo measurement. 
			clock_cycles_ctr=Get_system_register(AVR32_COUNT);	
			correct_navigation_states();
			clock_cycles_ctr=Get_system_register(AVR32_COUNT)-clock_cycles_ctr;
			clock_cycles_vec[4]=clock_cycles_ctr;
	
	
			//Calculate the a posteriori state covariance matrix
			clock_cycles_ctr=Get_system_register(AVR32_COUNT);	
			measurement_update();		
			clock_cycles_ctr=Get_system_register(AVR32_COUNT)-clock_cycles_ctr;
			clock_cycles_vec[5]=clock_cycles_ctr;		
		}
		else
		{	
			// If there was no zero-velocity update, just set the clock cycle counters to zero.
			clock_cycles_vec[3]=0;	
			clock_cycles_vec[4]=0;
			clock_cycles_vec[5]=0;	
		}
	
		/********************************/
	
		//Write the results to Matlab
		write_nav_data_to_matlab();
		write_cov_vector_data_to_matlab();
		write_clock_cycles_to_matlab();
		write_zupt_flag_to_matlab();
		}		
	}				
}


/*********************** FUNCTIONS *********************************/

void main_vbus_action(bool b_high)
{
	if (b_high) {
		udc_attach();
	} else {
		udc_detach();
	}
}

static void mdelay(unsigned int ms)
{
	int32_t count, count_end;

	count = Get_system_register(AVR32_COUNT);
	count_end = count + ((48000000 + 999) / 1000) * ms;
	while ((count_end - count) > 0)
		count = Get_system_register(AVR32_COUNT);
}

void get_imu_data_from_matlab(void){
	uint8_t data_ctr;
	
	// Read the accelerations
		for(data_ctr = 0; data_ctr <(sizeof(accelerations_in)/sizeof(precision));data_ctr++)
		 {
		 udi_cdc_read_buf((int*)(accelerations_in+data_ctr),sizeof(precision));        
	     }
		 
		 
	// Read the angular rates
		for(data_ctr = 0; data_ctr <(sizeof(angular_rates_in)/sizeof(precision));data_ctr++)
		 {
	  	 // Read the input data
		 udi_cdc_read_buf((int*)(angular_rates_in+data_ctr),sizeof(precision));       
	     }
		
}

void write_nav_data_to_matlab(void){
	uint8_t data_ctr=0;
	
		// Position data
	    for(data_ctr=0; data_ctr < (sizeof(position)/sizeof(precision));data_ctr++)
		{
		udi_cdc_write_buf((int*)(position+data_ctr),sizeof(precision));	  	
		}
		
		// Velocity data
	    for(data_ctr=0; data_ctr < (sizeof(velocity)/sizeof(precision));data_ctr++)
		{
		udi_cdc_write_buf((int*)(velocity+data_ctr),sizeof(precision));	  	
		}
		
		// Quaternions data
	    for(data_ctr=0; data_ctr <(sizeof(quaternions)/sizeof(precision));data_ctr++)
		{
		udi_cdc_write_buf((int*)(quaternions+data_ctr),sizeof(precision));	  	
		}
	
}

void write_cov_vector_data_to_matlab(void){
	uint8_t data_ctr=0;
	
		// Position data
	    for(data_ctr=0; data_ctr <(sizeof(cov_vector)/sizeof(precision));data_ctr++)
		{
		udi_cdc_write_buf((int*)(cov_vector+data_ctr),sizeof(precision));	  	
		}
		
}

void write_clock_cycles_to_matlab(void){
	uint8_t data_ctr=0;
		
		//Clock cycles data
	    for(data_ctr=0; data_ctr <(sizeof(clock_cycles_vec)/sizeof(uint32_t));data_ctr++)
		{
		udi_cdc_write_buf((int*)(clock_cycles_vec+data_ctr),sizeof(uint32_t));	  	
		}
}

void get_settings_from_matlab(void){
	
uint8_t tmp=0;

// Wait for start commando
while (tmp !='S')
{
	udi_cdc_read_buf((int*)&tmp,sizeof(tmp));	
}

// write the (size of) precision to Matlab 
udi_cdc_putc((int)sizeof(precision));


/************* GENERAL PARAMETERS ******************/

// Initial position [m]
udi_cdc_read_buf((int*)(initial_pos),sizeof(initial_pos));       

//Initial heading [rad] 
udi_cdc_read_buf((int*)(&initial_heading),sizeof(precision));

// Latitude [deg]
udi_cdc_read_buf((int*)(&latitude),sizeof(precision));

// Altitude [m]
udi_cdc_read_buf((int*)(&altitude),sizeof(precision));

//Sampling period [s]
udi_cdc_read_buf((int*)(&dt),sizeof(precision));


	
/************* ALIGNMENT PARAMETERS   **************/         

//Number of samples used in the initial alignment 
udi_cdc_read_buf((int*)(&nr_of_inital_alignment_samples),1);




/*********** Detector Settings ************/

// Accelerometer noise std used in the ZUPT detector [m/s^2]
udi_cdc_read_buf((int*)(&sigma_acc_det),sizeof(precision));

// Gyroscope noise std used in the ZUPT detector [rad/s]
udi_cdc_read_buf((int*)(&sigma_gyro_det),sizeof(precision));

// Window size used in the ZUPT detector [samples]
udi_cdc_read_buf((int*)(&detector_Window_size),1);

// Threshold used in the ZUPT detector 
udi_cdc_read_buf((int*)(&detector_threshold),sizeof(precision));



/************ FILTER PARAMETERS ************/

// Accelerometer process noise std [m/s^2]
udi_cdc_read_buf((int*)(&sigma_acceleration),sizeof(precision));


// Gyroscope process noise std [rad/s]
udi_cdc_read_buf((int*)(&sigma_gyroscope),sizeof(precision));


// Zero velocity update pseudo measurement noise std [m/s] 
udi_cdc_read_buf((int*)(sigma_velocity),sizeof(sigma_velocity));


// Initial position uncertainty (standard deviations) [m]  
udi_cdc_read_buf((int*)(sigma_initial_position),sizeof(sigma_initial_position));

	
// Initial velocity uncertainty (standard deviations) [m/s] 
udi_cdc_read_buf((int*)(sigma_initial_velocity),sizeof(sigma_initial_velocity));

// Initial attitude uncertainty (standard deviations) [rad/s]  
udi_cdc_read_buf((int*)(sigma_initial_attitude),sizeof(sigma_initial_attitude));
}

void write_zupt_flag_to_matlab(void){
	
// write the state of the zupt_flag to matlab
udi_cdc_write_buf((int*)(&zupt),1);	
}