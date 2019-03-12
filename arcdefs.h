//
// Define our types here
//

#include <robotcontrol.h>

// Some defines that we would like to use

#define		ARC_ON			1
#define		ARC_OFF			0
#define		ARC_FAIL		1
#define		ARC_PASS		0
#define		ARC_SWAP		-1.0
#define		ARC_NO_SWAP		1.0
#define		ARC_PI			3.14159
#define		M_TO_INCH		39.37
#define		INCH_TO_M		(1.0/39.37) 
#define		ARC_TRUE		1
#define		ARC_FALSE		0
#define    	ARC_FWD        		1
#define   	ARC_BWD        		-1
#define    	ARC_RAD2DEG     	57.2958

// ******************************************
// Structure to hold PID gains
// ******************************************

typedef struct  arc_PIDgain_t {
    double   Kp ;
    double   Ki ;
    double   Kd ;
} arc_PIDgain_t ;


// *******************************************
// Structure to hold motor data
// ******************************************

typedef struct  arc_motor_t {
    rc_filter_t     *pid ;           // Pointer to a filter to be used for PID control
    int             id ;             // Motor number {1, 2, 3, 4}
    int             swap ;           // -1 = swap blk and red wires, 1 is don't swap
    int             dir ;            // 1 = forward and -1 = backward
    int             setpoint ;       // PID setpoint
} arc_motor_t ;

// *********************************************
// Structure to hold location data
// *********************************************

typedef struct  arc_location_t {
    double       x ;			// x-coordinate (in inches)
    double       y ;			// y-coordinate (in inches)
    double       theta ;		// orientation (pi / 2 for looking north)
} arc_location_t ;
    
// *********************************************    
// Structure to hold robot config data
// *********************************************

typedef struct arc_config_t {
    double          sample_rate ;			       // Sampling frequency
    double          Ts ;			               // Period used for updates
    double          r ;			                   // Wheel radius in inches
    double          d ;			                   // Spacing between wheels in inches
    double          encoder_tics_per_revolution ;
    double          inches_per_tic ;	
    double          tics_per_inch ;
    double          max_pwm ;
    arc_PIDgain_t   motor_PID_gain ; 			   // PID gain constants for motor
    rc_mpu_config_t mpu_config ;			       // MPU config
} arc_config_t ;

// ********************************************
// Structure which defines our robot
// ********************************************

typedef struct  arc_robot_t {
    arc_config_t    config ;                // Struct that contains our robot configuration
    arc_motor_t     right_motor ;           // Struct for the right motor
    arc_motor_t     left_motor ;            // Struct for the left motor
    arc_location_t  location ;              // Robot's current location
    double          velocity ;              // Velocity of robot we desire in in / sec
    double          state ;                 // Robot state
    int             greenLED ;              // State of the green LED
    int             redLED ;                // State of the red LED
    rc_mpu_data_t   mpu_data;	            // MPU data
} arc_robot_t ;

