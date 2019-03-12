//
// arclib is a library of robot routimes 
// makes heavy use of strawson libcontrol
//

#include    <stdio.h>
#include    <math.h>
#include    <robotcontrol.h>
#include    <rc/math.h>

#include   "arcdefs.h"
 
extern  arc_robot_t  robot ;

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to configure robot
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^

int arc_config(void) {

    double    perimeter ;

// Robot configuration settings

    robot.config.sample_rate = 50.0 ;                   		// Hz
    robot.config.Ts = 1.0 / robot.config.sample_rate ;	        // sec
    robot.config.r = 1.375 ;                                    // wheel radius
    robot.config.d = 5.25 ;                 	                // distance between wheels
    robot.config.encoder_tics_per_revolution = 1440.0 ; 
    robot.config.motor_PID_gain.Kp = 0.005 ;				    // Proportional gain constant
    robot.config.motor_PID_gain.Ki = 0.0005 ;				    // Integral gain constant
    robot.config.motor_PID_gain.Kd = 0 ;  			            // Differential gain constant

// Right motor configuration

   robot.right_motor.id = 2 ;			        // Blue motor number
   robot.right_motor.swap = ARC_SWAP ;	        // No need to swap motor wires

// Left motor configuration

   robot.left_motor.id = 3 ;                    // Blue motor number
   robot.left_motor.swap = ARC_NO_SWAP ;        // Swap blk and red wires

// Config common to both motors

    perimeter = 2.0 * M_PI * robot.config.r ;
    robot.config.tics_per_inch = robot.config.encoder_tics_per_revolution / perimeter ;
    robot.config.inches_per_tic = 1.0 / robot.config.tics_per_inch ;

    return ARC_PASS ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to initialize everything
// and set our current location and orientation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int arc_init(arc_location_t loc) {

    double	Kp, Ki, Kd, dt ;

// Set up the filters we need for PID on right and left motors

    Kp = robot.config.motor_PID_gain.Kp ;
    Ki = robot.config.motor_PID_gain.Ki ;
    Kd = robot.config.motor_PID_gain.Kd ;
    dt = robot.config.Ts ;

// Create the right motor PID filter

    if(rc_filter_pid(robot.right_motor.pid, Kp, Ki, Kd, (4.0 * dt), dt)){
        fprintf(stderr,"ERROR in arc_init(), failed to make PID right motor controller filter.\n");
        return ARC_FAIL ;
    }

// Create the left motor PID filter

    if(rc_filter_pid(robot.left_motor.pid, Kp, Ki, Kd, (4.0 * dt), dt)){
	fprintf(stderr,"ERROR in arc_init(), failed to make PID left motor controller filter.\n");
	return ARC_FAIL ;
    }

//	Set our current location

    robot.location = loc ;		// x, y, theta

// Initialize motors

	if (rc_motor_init_freq(RC_MOTOR_DEFAULT_PWM_FREQ)) {
		return ARC_FAIL;	// Set PWM frequency
	}

    if (rc_motor_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize motors\n");
		return ARC_FAIL;
	}

//	rc_motor_standby(1); // start with motors in standby

// Initialize enocders

	if (rc_encoder_eqep_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
		return ARC_FAIL;
	}

// Start MPU

/*
	if(rc_mpu_initialize_dmp(&(robot.mpu_data), robot.config.mpu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return ARC_FAIL;
	}
*/
// If gyro isn't calibrated, run the calibration routine

/*
	if(!rc_mpu_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(robot.config.mpu_config);
	}
*/

// Turn the green LED ON, red LED OFF

    rc_led_set(RC_LED_GREEN, ARC_ON);
    robot.greenLED = ARC_ON ;
    rc_led_set(RC_LED_RED, ARC_OFF);
    robot.redLED = ARC_OFF ;

// Put the motors into freespin mode

//    rc_motor_free_spin(robot.right_motor.id);
//    rc_motor_free_spin(robot.left_motor.id);

// Return

	return ARC_PASS ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine used to update our location
// Accepts current location along with right tic
// and left motor tic values
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

arc_location_t  arc_update_location(arc_location_t  cur_loc, int ticsR, int ticsL) {
   arc_location_t   loc ;
   double           delta_s, delta_theta ;
   double           delta_x, delta_y ;

// Compute our current location

   delta_s = 0.5 * (ticsR + ticsL) ;
   delta_s *= robot.config.inches_per_tic ;

   delta_theta = ticsR - ticsL ;
   delta_theta *= robot.config.inches_per_tic ;
   delta_theta /= robot.config.d ;

   loc.theta = cur_loc.theta + delta_theta ;
   delta_x = delta_s * cos(cur_loc.theta) ;
   delta_y = delta_s * sin(cur_loc.theta) ;

   delta_y = delta_s ;

   loc.x = cur_loc.x + delta_x ;
   loc.y = cur_loc.y + delta_y ;

// Return our new location on the track

   return  loc ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to "goto" given location
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int  arc_goto(arc_location_t target_loc) {
    int	            target_reached ;
    int	            cntR, cntL, old_cntR, old_cntL, spR, spL ;
    double          ticsR, ticsL, pwmR, pwmL ;
    int	            error ;  
    double          tmp ;
    arc_location_t  loc ;

// Set flag for target reached location

   target_reached = ARC_FALSE ;

// Set current robot location

   loc = robot.location ;

// Reset the PID filters
// Enable saturation and ask for a slow start in the PID filters

    rc_filter_reset(robot.right_motor.pid) ;
    rc_filter_reset(robot.left_motor.pid) ;    
    rc_filter_enable_saturation(robot.right_motor.pid, 0, robot.config.max_pwm) ;
    rc_filter_enable_saturation(robot.left_motor.pid, 0, robot.config.max_pwm) ;
    tmp = 5.0 * robot.config.Ts ;
    rc_filter_enable_soft_start(robot.right_motor.pid, tmp) ;
    rc_filter_enable_soft_start(robot.left_motor.pid, tmp) ;

// PWM intialization

    pwmR = 0.0 ; pwmL = 0.0 ;

// Set motor directions

   robot.right_motor.dir = ARC_FWD ;
   robot.left_motor.dir = ARC_FWD ;

// Encoder initialization
// Set encoder values to 0

    rc_encoder_write(robot.right_motor.id, 0) ;
    rc_encoder_write(robot.left_motor.id, 0) ;
    cntR = 0 ; cntL = 0 ;
    
// Compute number of tics we expect to see in sample period
// This is the setpoint into the PID controller

    tmp = robot.config.Ts * robot.velocity ;
    tmp = tmp / robot.config.inches_per_tic ;
    spR = (int) (tmp + 0.5) ;
    spL = spR ;
    robot.right_motor.setpoint = spR ;
    robot.left_motor.setpoint = spL ;

// Keep going until target reached

    while (target_reached == ARC_FALSE) {

// Update pwm values for motors
// We might need to "swap" black and red wires

       pwmR *= robot.right_motor.swap * robot.right_motor.dir ;
       pwmL *= robot.left_motor.swap * robot.left_motor.dir  ;

       rc_motor_set(robot.right_motor.id, pwmR);
       rc_motor_set(robot.left_motor.id, pwmL);

// Read right and left motor encoders
// Compute how far each wheel moved in "tics"
// during previous sample period.

       old_cntR = cntR ; 
       old_cntL = cntL ;
       cntR = rc_encoder_read(robot.right_motor.id) ;
       cntR *= robot.right_motor.swap ;
       cntL= rc_encoder_read(robot.left_motor.id) ;
       cntL *= robot.left_motor.swap ;
       ticsR = cntR - old_cntR ; 
       ticsL = cntL - old_cntL ;

// Update our location

       loc = arc_update_location(loc, ticsR, ticsL) ;

// Check to see if we have reached the target 

       if (loc.y > target_loc.y) target_reached = ARC_TRUE ;

// Compute error and filter
// Error is the difference between how many tics we actually moved
// and what we had expected to move.

       error = (double) (spR - ticsR) ;
       pwmR = rc_filter_march(robot.right_motor.pid, error) ;
       error = (double) (spL - ticsL) ;
       pwmL = rc_filter_march(robot.left_motor.pid, error) ;

// Toggle the green LED to inform user that we are in the goto routine

       if (robot.greenLED == ARC_ON) {
             robot.greenLED = ARC_OFF ;
	         rc_led_set(RC_LED_GREEN, ARC_OFF);
        } else {
             robot.greenLED = ARC_ON ;
	         rc_led_set(RC_LED_GREEN, ARC_ON);
        } 
              
// Check to make sure no one has tried to Control-C out of the program
// If not then wait 50 ms before continuing
  
       if (rc_get_state() != EXITING) {
           rc_usleep(5000);
       } else {
            return ARC_PASS ;
       }
    } // end while

// Update robot locaction entry

    robot.location = loc ;

    return ARC_PASS ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to cleanup after ourselves
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int arc_cleanup(void) {

// Clean up motors

	rc_motor_cleanup();

// Free up the memory used by the PID filters

	rc_filter_free(robot.right_motor.pid);
	rc_filter_free(robot.left_motor.pid);

// Turn the LEDs off and shutoff LED handlers

	rc_led_set(RC_LED_GREEN, ARC_OFF);
	rc_led_set(RC_LED_RED, ARC_OFF);
	rc_led_cleanup();

// Cleanup the encoder stuff

	rc_encoder_eqep_cleanup() ;	

// Turn mpu off

//	rc_mpu_power_off();

	return ARC_PASS ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to dump key values
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void arc_var_dump(void) {
    printf("Sample rate is %g Hz\n", robot.config.sample_rate) ;
    printf("Sample period is %g ms\n", 1000.0 * robot.config.Ts) ;
    printf("Wheel radius is %g in\n", robot.config.r) ;
    printf("Encoder tics per wheel revolution is %g\n", robot.config.encoder_tics_per_revolution) ;
    printf("Inches per tic is %g\n", robot.config.inches_per_tic) ;
    printf("Tics per inch is %g\n", robot.config.tics_per_inch) ;
    printf("Maximum pwm value is %g\n", robot.config.max_pwm) ;
    printf("Velocity of robot is %g in/sec\n", robot.velocity) ;
    printf("Right motor setpoint is %d\n", robot.right_motor.setpoint) ;
    printf("Left motor setpoint is %d\n", robot.left_motor.setpoint) ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine print location
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void arc_print_location(arc_location_t loc) {
    printf("x is %g in\n", loc.x) ;
    printf("y is %g in\n", loc.y) ;
    printf("theta is %g degrees\n", loc.theta * ARC_RAD2DEG) ;
}
