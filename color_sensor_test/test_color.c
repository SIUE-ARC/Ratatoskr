//
// @file    test_color.c
//
//
// Will prove we can talk to I2C devices
//
// Need access to std i/o routines 
//

#include <stdio.h>

// Robotic Control Library
#include <robotcontrol.h>

// Color sensor related defines
#include "color_sensor.h"

// We need our i2c mux routines

#include   "i2c_mux.h"

//
// This is a program to test out a series of submodules
// that might be useful for robotics
//

#define	  ON       1
#define   OFF      0

#define   COLOR_SENSOR_MUX_PORT    4

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Main program
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int main(void) {

// Check for existing processes

    if(rc_kill_existing_process(2.0)<-2) return -1;

// Create a lock file

    rc_make_pid_file() ;

// Start signal handler so we can exit cleanly

   if(rc_enable_signal_handler()==-1) {
       fprintf(stderr,"ERROR: failed to start signal handler\n");
       return -1;
    }

// Set the state to running

   rc_set_state(RUNNING) ;

// I2C bus will get initialized

   rc_i2c_init(MUX_I2C_BUS, MUX_I2C_ADDR) ;

// Select the color sensor

   disableMuxPort(ALL_PORTS) ;
   enableMuxPort(COLOR_SENSOR_MUX_PORT) ;

// Initialize the color sensor

   init_color_sensor() ;

// Turn red and green LEDs off
// Turn on the green one when we detect the green LED

   rc_led_set(RC_LED_GREEN, 0) ;
   rc_led_set(RC_LED_RED, 0) ;

// Read the color sensor

   unsigned int  c, r, g, b ;

   int   do_it = 1 ;
   while (do_it < 30) {
       read_color_sensor(&c, &r, &g, &b) ;
       printf("\nClear data value: %u\n", c) ;
       printf("Red data value: %u\n", r) ;
       printf("Green data value: %u\n", g) ;
       printf("Blue data value: %u\n\n", b) ;
       if (g > 1400) {
           rc_led_set(RC_LED_GREEN, ON) ;
           rc_led_set(RC_LED_RED, OFF) ;
       } else {
           rc_led_set(RC_LED_GREEN, OFF) ;
           rc_led_set(RC_LED_RED, ON) ;
       }
       do_it += 1 ;
       if (rc_get_state() == EXITING) break ;
       rc_usleep(3000000) ;
   } // end while
    
// Closing up the color sensor

   cleanup_color_sensor() ; 

// Disable all the MUX ports

   rc_i2c_set_device_address(MUX_I2C_BUS, MUX_I2C_ADDR) ;
   disableMuxPort(ALL_PORTS) ;

// Close the i2c channel

   rc_i2c_close(MUX_I2C_BUS) ;

// Remove the lock file

   rc_remove_pid_file() ;	

   return 0 ;
} // end main

