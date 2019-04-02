//
// @file    gpio_test.c
//
// Program to test creating a timing loop
// 
// Use servo output to output a pulse every 50 ms
// Connect servo output to an input pin.
// Poll the input pin and toggle the green LED
//
// Use the adafruit servo board which sits on
// I2C bus.  
//
// Also will prove we can talk to I2C devices
//

#include    <stdio.h>
#include    <signal.h>
#include    <getopt.h>
#include    <math.h>
#include    <robotcontrol.h>

// Use pin 1 from GPIO3 block

#define     CHIP_NUM     3
#define     PIN_NUM      1

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// MAIN PROGRAM
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

int  main() {
    int   greenLED ;
    int   pin_val ;

	if(rc_kill_existing_process(2.0)<-2) return -1;

// Create a lock file

	rc_make_pid_file();

// Start signal handler so we can exit cleanly

	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

// Turn the green LED ON, red LED OFF

    rc_led_set(RC_LED_RED, 0);
    greenLED = 1 ;
    rc_led_set(RC_LED_GREEN, greenLED);

// Initilize the gpio
    
    rc_gpio_init(CHIP_NUM, PIN_NUM, GPIOHANDLE_REQUEST_INPUT) ;

// Set the system state to RUNNING
// If user issues a Ctrl-C then the state will change to EXITING

	rc_set_state(RUNNING) ;

// Infinite loop

    while (1) {
 
// Wait for the pin to come high and toggle green LED

          while(rc_gpio_get_value(CHIP_NUM, PIN_NUM) == 0) { } ;

// Toggle 
          
          if (greenLED == 1) greenLED = 0 ;
          else greenLED = 1 ;

          rc_led_set(RC_LED_GREEN, greenLED);         

// Wait for the pin to go low   
       
          while(rc_gpio_get_value(CHIP_NUM, PIN_NUM) == 1) { } ;

// Here is our way out of the infinite loop

          if (rc_get_state() == EXITING) break ;

   } // end infinite loop


// Cleanup

   rc_gpio_cleanup(CHIP_NUM, PIN_NUM) ;

// Remove the lock file

	rc_remove_pid_file();	

} 

