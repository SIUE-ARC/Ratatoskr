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

// Servo driver specific defines

#include "servo_driver.h"

// Need access to the I2C Mux

#include   "i2c_mux.h"

// Simple main program which creates servo control signals

int main(void) {

// Check for existing processes

    if(rc_kill_existing_process(2.0)<-2) return -1;

// Create a lock file

    rc_make_pid_file();

// Start signal handler so we can exit cleanly

    if(rc_enable_signal_handler()==-1){
       fprintf(stderr,"ERROR: failed to start signal handler\n");
       return -1;
    }

// Turn on the servo power rail
// We will use this rail to power out I2C servo driver IC

   rc_servo_init() ;
   rc_servo_power_rail_en(1) ;

// Initialize the i2c bus

   rc_i2c_init(SERVO_I2C_BUS, SERVO_I2C_ADDR) ;

// Disable all of the I2C Mux ports

   disableMuxPort(ALL_PORTS) ;

// Reset our servo driver IC

    resetServoDriver() ;

// Set rep rate to 50 Hz

    setServoFREQ(50.0) ;

// Turn servo one direction and then back the other

    setServoPW(0, 150) ;
    rc_usleep(3000000) ;

    setServoPW(0, 350) ;
    rc_usleep(3000000) ;

    setServoPW(0, 150) ;
    rc_usleep(3000000) ;

    setServoPW(0, 350) ;
    rc_usleep(3000000) ;


// Reset our servo driver

   resetServoDriver() ;

// Cleanup

   rc_i2c_close(SERVO_I2C_BUS) ;

// Turn off the servo power rail

   rc_servo_cleanup() ;

// Remove the lock file

   rc_remove_pid_file();	

   return(0);
}


