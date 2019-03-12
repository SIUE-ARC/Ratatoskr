//
// @file    bluebot.c
//
// Simple robot program using beaglebone blue
//

#include    <stdio.h>
#include    <signal.h>
#include    <getopt.h>
#include    <rc/motor.h>
#include    <rc/time.h>
#include    <math.h>
#include    <robotcontrol.h>

#include    "arclib.h"

//
// Create our robot structure as a global
// That way all our arc routines will have easy 
// access to it 
//

arc_robot_t     robot ;

// int             debug = ARC_TRUE ;

rc_filter_t     left_motor_filter = RC_FILTER_INITIALIZER ;
rc_filter_t     right_motor_filter = RC_FILTER_INITIALIZER ;

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// MAIN PROGRAM
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

int  main() {
   arc_location_t  loc ;

// Enter pointers to filters into our robot structure
// Our PID filters are defined as globals.

   robot.right_motor.pid = &(right_motor_filter) ;
   robot.left_motor.pid = &(left_motor_filter) ;

// make sure another instance isn't running
// if return value is -3 then a background process is running with
// higher privaledges and we couldn't kill it, in which case we should
// not continue or there may be hardware conflicts. If it returned -4
// then there was an invalid argument that needs to be fixed.

	if(rc_kill_existing_process(2.0)<-2) return -1;

// Create a lock file

	rc_make_pid_file();

// Start signal handler so we can exit cleanly

	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

// Set the system state to RUNNING
// If user issues a Ctrl-C then the state will change to EXITING

	rc_set_state(RUNNING) ;

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Configure our robot
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    arc_config() ;   

// Set the maximum pwm < 1.0

    robot.config.max_pwm = 0.9 ;      

// Set starting location and orientation

    loc.x = 0 ; loc.y = 0 ; loc.theta = 0  ; 

    if (arc_init(loc) == ARC_FAIL) {
		fprintf(stderr,"ERROR: failed to initialize robot. Aborting\n");
		return -1 ;  
	} 

// Set target location, orientation, and robot velocity

    loc.x = 0; loc.y = 0.0 ; loc.theta = 0  ;
    robot.velocity = 5.0 ;

// Goto target location

    arc_goto(loc) ; 

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Cleanup before exiting
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    arc_cleanup() ;

// Remove the lock file

	rc_remove_pid_file();	

// Print loation of robot

   arc_print_location(robot.location) ;

// Variable dump

   arc_var_dump() ;

} 

