//
// @file    test_servo.c
//
//
// Will prove we can talk to I2C devices
//
// Need access to std i/o routines 
//

#include <stdio.h>
#include <robotcontrol.h>
#include "servo_driver.h"

// Simple main program which gets a reading from the SRF02

int main(void) {
   int i ;

// Check for existing processes

    if(rc_kill_existing_process(2.0)<-2) return -1;

// Create a lock file

    rc_make_pid_file();

// Start signal handler so we can exit cleanly

	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

// Initialize the i2c bus

    rc_i2c_init(I2C_BUS, SERVO_I2C_ADDR) ;

// Turn on the servo power rail

   rc_servo_init() ;
   rc_servo_power_rail_en(1) ;

// Reset the servo driver

    resetServoDriver() ;

// Set rep rate to 50 Hz

    setServoFREQ(50.0) ;

// Turn servo one direction and then back the other

    for (i=150; i<=350; i+=50) {
       setServoPW(0, i) ;
       rc_usleep(900000) ;
       rc_usleep(900000) ;
     }

   resetServoDriver() ;

// Cleanup

    rc_i2c_close(I2C_BUS) ;

// Turn off the servo power rail

   rc_servo_cleanup() ;


// Remove the lock file

   rc_remove_pid_file();	

   return(0);
}


