//
// arclib is a library of robot routimes 
// makes heavy use of strawson libcontrol
//

#include    <stdio.h>
#include    <stdlib.h>
#include    <math.h>
#include    <robotcontrol.h>
#include    <unistd.h>
#include    <fcntl.h>
//#include    <rc/servo.h>
#include    "arcdefs.h"

extern  arc_robot_t  robot ;
extern  rc_filter_t  left_motor_filter ;
extern  rc_filter_t  right_motor_filter ;
extern  bool         DEBUG ;



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
    printf("Right motor setpoint is %d\n", robot.right_motor.sp) ;
    printf("Left motor setpoint is %d\n", robot.left_motor.sp) ;
} // end arc_dump()

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine print location
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void arc_print_location(arc_location_t loc) {
    printf("x is %g in\n", loc.xy.d[X]) ;
    printf("y is %g in\n", loc.xy.d[Y]) ;
    printf("theta is %g degrees\n", loc.theta * ARC_RAD2DEG) ;
} // end arc_print_location()

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to initialize everything
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int arc_init(void) {

    double	Kp, Ki, Kd, dt ;

// Set up the filters we need for PID on right and left motors

    Kp = robot.config.motor_PID_gain.Kp ;
    Ki = robot.config.motor_PID_gain.Ki ;
    Kd = robot.config.motor_PID_gain.Kd ;
    dt = robot.config.Ts ;

// Create the right and left motor PID filter

    if(rc_filter_pid(robot.right_motor.pid, Kp, Ki, Kd, (4.0 * dt), dt)){
        fprintf(stderr,"ERROR in arc_init(), failed to make PID right motor controller filter.\n");
        return ARC_FAIL ;
    }
    if(rc_filter_pid(robot.left_motor.pid, Kp, Ki, Kd, (4.0 * dt), dt)){
	fprintf(stderr,"ERROR in arc_init(), failed to make PID left motor controller filter.\n");
	return ARC_FAIL ;
    }

// Initialize motors and encoders / servos

    if (rc_motor_init_freq(RC_MOTOR_DEFAULT_PWM_FREQ)) {
        return ARC_FAIL;	// Set PWM frequency
    }
    if (rc_motor_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize motors\n");
        return ARC_FAIL;
    }

    if (rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return ARC_FAIL;
    }

// Turn both LEDs OFF

    rc_led_set(RC_LED_GREEN, ARC_OFF);
    robot.greenLED = ARC_OFF ;
    rc_led_set(RC_LED_RED, ARC_OFF);
    robot.redLED = ARC_OFF ;
    return ARC_PASS ;

} // end arc_init() 

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to convert degrees to radians
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

double arc_deg2rad(double deg) {
    return  ((M_PI / 180.0) * deg) ;
} // end arc_deg2rad()

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to convert radians to degrees
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

double arc_rad2deg(double rad) {
    return  ((180.0 / M_PI) * rad) ;
} // end arc_rad2deg()


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to perform some general setup
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int  arc_setup(void) {

// Enter pointers to filters into our robot structure
// Our PID filters are defined as globals.

   robot.right_motor.pid = &(right_motor_filter) ;
   robot.left_motor.pid = &(left_motor_filter) ;

// make sure another instance isn't running
// if return value is -3 then a background process is running with
// higher privaledges and we couldn't kill it, in which case we should
// not continue or there may be hardware conflicts. If it returned -4
// then there was an invalid argument that needs to be fixed.

   if (rc_kill_existing_process(2.0)<-2) return ARC_FAIL ;

// Create a lock file

   rc_make_pid_file() ;

// Start signal handler so we can exit cleanly

   if (rc_enable_signal_handler()==-1) {
       fprintf(stderr,"ERROR: failed to start signal handler\n");
       return ARC_FAIL ;
    }

// Set the system state to RUNNING
// If user issues a Ctrl-C then the state will change to EXITING

   rc_set_state(RUNNING) ;
   return ARC_PASS ;
} // end general setup 


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to configure robot
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^

int arc_config(void) {

    double    perimeter ;

// Robot configuration settings

    robot.config.sample_rate = 20   ;                                // Hz
    robot.config.Ts = 1.0 / robot.config.sample_rate ;	             // sec
    robot.config.Ts_in_ns = (uint64_t) (robot.config.Ts * 1e9)   ;
    robot.config.r = 1.415 ;                                         // wheel radius
    robot.config.d = 5.83 ;                 	                     // distance between wheels
    robot.config.encoder_tics_per_revolution = 1440.0 ; 
    robot.config.motor_PID_gain.Kp = 0.005 ;	                     // Proportional gain constant
    robot.config.motor_PID_gain.Ki = 0.0005 ;		             // Integral gain constant
    robot.config.motor_PID_gain.Kd = 0 ;                             // Differential gain constant

// Right motor configuration

   robot.right_motor.id = 1 ;			// Blue motor number
   robot.right_motor.swap = ARC_SWAP ;	        //  swap motor wires

// Left motor configuration

   robot.left_motor.id = 2 ;                    // Blue motor number
   robot.left_motor.swap = ARC_NO_SWAP ;        // dont't swap blk and red wires

// Config common to both motors

    perimeter = 2.0 * M_PI * robot.config.r ;
    robot.config.tics_per_inch = robot.config.encoder_tics_per_revolution / perimeter ;
    robot.config.inches_per_tic = 1.0 / robot.config.tics_per_inch ;
    robot.config.max_pwm = 0.9 ;

// Set current and target locations

    robot.location.theta = 0.0 ;
    robot.location.s = 0.0 ;
    robot.location.xy = RC_VECTOR_INITIALIZER ;
    rc_vector_zeros(&robot.location.xy, 2) ;

    robot.target.theta = 0.0 ;
    robot.target.s = 0.0 ;
    robot.target.xy = RC_VECTOR_INITIALIZER ;
    rc_vector_zeros(&robot.target.xy, 2) ;

    return ARC_PASS ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to cleanup after ourselves
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int arc_cleanup(void) {

// Clean up motors / servos

    rc_motor_cleanup();
    rc_servo_cleanup();

// Free up the memory used by the PID filters

    rc_filter_free(robot.right_motor.pid);
    rc_filter_free(robot.left_motor.pid);

// Turn the LEDs off and shutoff LED handlers

    rc_led_set(RC_LED_GREEN, ARC_OFF);
    rc_led_set(RC_LED_RED, ARC_OFF);
    rc_led_cleanup();

// Cleanup the encoder stuff

    rc_encoder_eqep_cleanup() ;

// Remove the lock file

    rc_remove_pid_file();	

// Print loation of robot and dump variables

/*
       arc_print_location(robot.location) ;
       arc_var_dump() ;
*/
   return ARC_PASS ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine used to update our location
// Accepts current location along with right tic
// and left motor tic values
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void  arc_update_location(void) {
   double           delta_s, delta_theta ;
   double           delta_x, delta_y ;
   double           ticsR, ticsL ;

   ticsR = robot.right_motor.tics ;
   ticsR *= robot.right_motor.dir ;

   ticsL = robot.left_motor.tics ;
   ticsL *= robot.left_motor.dir ;

   delta_s = 0.5 * (ticsR + ticsL) ;
   delta_s *= robot.config.inches_per_tic ;
   robot.location.s += delta_s ;

   delta_theta = ticsR - ticsL ;
   delta_theta *= robot.config.inches_per_tic ;
   delta_theta /= robot.config.d ;
   robot.location.theta += delta_theta ;

   delta_x = delta_s * cos(robot.location.theta) ;
   delta_y = delta_s * sin(robot.location.theta) ;
   robot.location.xy.d[X] += delta_x ;
   robot.location.xy.d[Y] += delta_y ;  
 
   return ;
} // end arc_update_location() 

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine used to toggle green LED
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void toggleGreenLED(void) {

   if (robot.greenLED == ARC_ON) {
      robot.greenLED = ARC_OFF ;
      rc_led_set(RC_LED_GREEN, ARC_OFF);
   } else {
      robot.greenLED = ARC_ON ;
      rc_led_set(RC_LED_GREEN, ARC_ON);
   } 
     return ;
} // end toggleGreenLED() 

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine used by arc_forward and arc_rotate for initialization
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void  arc_move_init(int right_dir, int left_dir, bool soft_start) 
{
   double   td, tmp ;

// Reset the PID filters
// Enable saturation and ask for a slow start in the PID filters

    rc_filter_reset(robot.right_motor.pid) ;
    rc_filter_reset(robot.left_motor.pid) ;    
    rc_filter_enable_saturation(robot.right_motor.pid, 0, robot.config.max_pwm) ;
    rc_filter_enable_saturation(robot.left_motor.pid, 0, robot.config.max_pwm) ;
    if (soft_start) {
       td = 2.0 * robot.config.Ts ;
       rc_filter_enable_soft_start(robot.right_motor.pid, td) ;
       rc_filter_enable_soft_start(robot.left_motor.pid, td) ;
    }

// Set motor directions

    robot.right_motor.dir = right_dir ;
    robot.left_motor.dir = left_dir ;

// Encoder initialization
// Set encoder values to 0

    rc_encoder_write(robot.right_motor.id, 0) ;
    rc_encoder_write(robot.left_motor.id, 0) ;

// Compute number of tics we expect to see in sample period
// This is the setpoint into the PID controller

    robot.right_motor.pwm = 0.0 ; 
    robot.left_motor.pwm = 0.0 ;
    robot.right_motor.cnt = 0 ; 
    robot.left_motor.cnt = 0 ;
    tmp = robot.config.Ts * robot.velocity ;
    tmp = tmp / robot.config.inches_per_tic ;
    robot.right_motor.sp = (int) (tmp + 0.5) ;
    robot.left_motor.sp = (int) (tmp + 0.5) ;
    robot.location.s = 0.0 ;

    return ;
} // end arc_move_init()

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to move robot 
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void  arc_move(void) {
     int	     error ;  

// Read right and left motor encoders
// Compute how far each wheel moved in "tics"
// during previous sample period.

    robot.right_motor.old_cnt = robot.right_motor.cnt ;
    robot.left_motor.old_cnt = robot.left_motor.cnt ;
 
    robot.right_motor.cnt = abs(rc_encoder_read(robot.right_motor.id)) ;
    robot.left_motor.cnt = abs(rc_encoder_read(robot.left_motor.id)) ;

    robot.right_motor.tics = robot.right_motor.cnt - robot.right_motor.old_cnt ;
    robot.left_motor.tics = robot.left_motor.cnt - robot.left_motor.old_cnt ;
 
// Update pwm values for motors
// We might need to "swap" black and red wires

    robot.right_motor.pwm *= robot.right_motor.swap * robot.right_motor.dir ;
    robot.left_motor.pwm *= robot.left_motor.swap * robot.left_motor.dir ;

    rc_motor_set(robot.right_motor.id, robot.right_motor.pwm) ;
    rc_motor_set(robot.left_motor.id, robot.left_motor.pwm) ;

// Compute error and filter
// Error is the difference between how many tics we actually moved
// and what we had expected to move.

    error = (double) (robot.right_motor.sp - robot.right_motor.tics) ;
    robot.right_motor.pwm = rc_filter_march(robot.right_motor.pid, error) ;

    error = (double) (robot.left_motor.sp - robot.left_motor.tics) ;
    robot.left_motor.pwm = rc_filter_march(robot.left_motor.pid, error) ;  

// Update our location
  
    arc_update_location() ;

// Toggle the green LED to inform user that we are in the move routine

    toggleGreenLED() ;

} // end arc_move()

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to rotate robot
// Angle value should be in radians
// It the amount we want to rotate by
// + is CCW and - is CW
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void  arc_rotate(double angle) {
    int             target_reached ;
    uint64_t        start_time ;
    unsigned int    delay_time ;
    double          err, max_err ;
    double          thetaChange;
    thetaChange = robot.location.theta;
 
    // robot.location.theta = 0.0 ;
   robot.velocity = ROTATE_VEL ;

// We will accept a specified amount of error in our final location

   max_err = arc_deg2rad(MAX_ANGLE_ERROR)  ; 

// Wheels must rotate in opposite direction for a rotation
   
   if (fabs(angle) <= max_err) return ;
   if (angle >= 0.0) arc_move_init(ARC_FWD, ARC_BWD, false) ;  // Counter clock-wise
   else arc_move_init(ARC_BWD, ARC_FWD, true) ;               // Clock-wise

// Keep "moving" until we get "very close" to our desired orientation
// Compute delay on the fly to ensure fixed sample period!

   target_reached = false ;
   while (target_reached == false) {  
      start_time = rc_nanos_since_epoch() ;     
      arc_move() ;
      err = angle - (robot.location.theta-thetaChange) ;
      if (fabs(err) < max_err) target_reached = true ;
      if (rc_get_state() != EXITING) {
         delay_time = (unsigned int) ( (robot.config.Ts_in_ns - (rc_nanos_since_epoch() - start_time)) );
         delay_time /= 1000 ;              
         rc_usleep(delay_time);
      } else return ;
   } // end while
   rc_motor_brake(0) ;
   rc_usleep(500000) ;
   return ;

} // end arc_rotate() ;

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine used to move forward
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void  arc_forward(double distance, double velocity) {
    bool            target_reached ;
    uint64_t        start_time ;
    unsigned int    delay_time ;
    double          err ;
    double          thetaChange;
    thetaChange = robot.location.theta; // -1
   
    robot.velocity = velocity ;
    arc_move_init(ARC_FWD, ARC_FWD, true) ;
    target_reached = false ;
    while (target_reached == false) {  
      start_time = rc_nanos_since_epoch() ;     
      arc_move() ;
      err = distance - robot.location.s ;
      if (fabs(err) < MAX_DIST_ERROR) target_reached = true ;
      if (rc_get_state() != EXITING) {
         delay_time = (unsigned int) ( (robot.config.Ts_in_ns - (rc_nanos_since_epoch() - start_time)) );
         delay_time /= 1000 ;              
         rc_usleep(delay_time);
      } else return ;
    } // end while
    rc_motor_brake(0) ;
//    printf("theta is %f \n", robo
    arc_rotate(-1.0 * ( robot.location.theta - thetaChange) ) ;
//    fflush(stdout) ;ation.theta - thetaChange)) ; // -2
    rc_motor_brake(0) ;
    rc_usleep(500000) ;
    return ;
} // end arc_forward() 

bool arc_go_to_angle(double angle) {
	// cur - pi  go - 3/2pi  currT = pi  curre - goal = -1/2 pi
	double currentTheta = fmod(robot.location.theta,2*M_PI);

	arc_rotate(angle-currentTheta);
	//arc_rotate(angle-currentTheta);
	// angle   |  curr     | output 
	//  pi     |    0      | -pi
	//  2pi    |    0      | -2pi
	//  1/2pi  |    3/2 pi | pi
	//  

	return true;
}
bool arc_go_to_goal(double goalX,double goalY) {
// rotate to goal 
	/*
	double thetaGoal;
	thetaGoal = atan2(y,x);
	if(thetaGoal < 0)
	{
		arc_go_to_angle(-((2*M_PI) + thetaGoal));
	}
	else {
		arc_go_to_angle(-thetaGoal);
	}
	arc_forward(sqrt(pow(x,2) + pow(y,2)),6);
	//arc_rotate(thetaGoal-robot.loc
	return true;
 	*/

	double deltaX,deltaY,beta,currentX,currentY;
	currentX = robot.location.xy.d[X];
	currentY = robot.location.xy.d[Y];
	deltaX = goalX-currentX;
	deltaY = goalY-currentY;
	beta = atan2(deltaY,deltaX);
	
	//printf("angle rotation to goal: %f \n",beta);
	arc_go_to_angle(beta);
	
	//printf("theta1: %f\n",robot.location.theta);
	arc_go_to_angle(beta);
	//printf("theta2: %f\n",robot.location.theta);
	arc_forward(sqrt(pow(deltaX,2) + pow(deltaY,2)),6);
	return true;
}


int  getBoxLocation() {


	char command_result[84];
	
	sprintf(command_result, "echo \"SENDNUDES\" > /home/debian/bin/CtoPy");
	system(command_result);
	char data[32];
	/*int fp;
 	
	fp = open("/home/debian/bin/CtoPy",O_WRONLY);
	
	write(fp, "SENDNUDES",10);
	close(fp);
	*/
	//rc_usleep(5000000) ;
	int fp2;
	fp2 = open("/home/debian/bin/PytoC",O_RDONLY);
	read(fp2,data,64);
	printf("Data from Socket: %s \n",data);
	char charCode;
	int  xCord;
	close(fp2); 
	sscanf(data, "%c %d",&charCode,&xCord);
	//printf("%c %d\n",charCode,xCord);
	
	// Y for data seen followed by int for x location
	if(charCode == 'Y') {
		printf("Block seen, X cord value: %d\n",xCord);
	
		return xCord;
	}
	if(charCode == 'N') {
		printf("No block seen\n");
		return -1000000;
	}
	return -200000;

}

int  getGreenLocation() {
	int fp;
 	char data[32];
	fp = open("CtoPy",O_WRONLY | O_SYNC);
	
	write(fp, "GREEN\n",10);
	close(fp);
	

	fp = open("PytoC",O_RDONLY);
	int  xCord;
	char charCode;
	
	
	read(fp,data,64);
	printf("Data from Socket: %s \n ",data);
	sscanf(data, "%c %d",&charCode,&xCord);
	close(fp);
	// Y for data seen followed by int for x location
	if(charCode == 'Y') {
		printf("Green Light seen, X cord value: %d\n",xCord);
		return xCord;
	}
	if(charCode == 'N') {
		printf("No Green Light seen\n");
		return -1000000;
	}
	return -2000000;

}


/*void armLower()
{
	printf("hello");
	rc_servo_send_pulse_normalized(1,1);
}

void armRaise()
{
	rc_servo_send_pulse_normalized(1,-.8);
}
void gripperOpen()
{
	rc_servo_send_pulse_normalized(2,0);
}
void gripperClose()
{
	rc_servo_send_pulse_normalized(2,-.9);
}
*/
