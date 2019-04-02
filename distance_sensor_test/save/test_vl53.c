/*
 @file test_vl53.c
*/

//
// Program to test our VL53L1X distance sensor
//

#include 	<stdio.h>
#include	<robotcontrol.h>
#include 	"VL53L1X.h"
#include        "CrossPlatformI2C.h"


// Define some things we need so that we can use I2C bus

#define 	I2C_BUS 		1

// Write buffer size

#define		BUF_SIZE    		32

// 
// Create a VL53L1X distance sensor object
//

static VL53L1X distanceSensor;


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to implement msec delays
// Not sure if this really work ...
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

/*
void delay(uint32_t msec) {
   rc_usleep(1000 * msec) ;
   return ;
}
*/

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to setup the I2C bus so that we can talk to the
// distance sensor
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int setup(void) {

// Check for existing processes

    if (rc_kill_existing_process(2.0)<-2) return -1 ;

// Create a lock file

    rc_make_pid_file() ;

// Start signal handler so we can exit cleanly

    if(rc_enable_signal_handler()==-1) {
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

// Set the state to running

    rc_set_state(RUNNING) ;

// Print a welcome message

    printf("VL53L1X Qwiic Test\n") ;

// Initialize the i2c bus

   cpi2c_open(0x29, I2C_BUS) ;

// Wait until we think the sensor has been connected
// The begin function will intialize the I2C bus
// It also configures the sensor.

    if (distanceSensor.begin(0x29) == false) {
        while (true) {
            printf("Sensor offline!\n");
            rc_usleep(9000000) ;
            if (rc_get_state() == EXITING) break ;
        } // end while
    } // end if
    return 1 ;
} // end setup()

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Main program 

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int main(void) {
    
    int i ;

// Run the setup function

    setup() ;

// Just sit in a for loop and print distance readings to the terminal   

    for (i = 1; i <= 5; i++ ) {

// Poll for completion of measurement. Takes 40-50ms.

       while (distanceSensor.newDataReady() == false) {
           printf("No new data ready.\n") ;
           rc_usleep(50000) ;
           if (rc_get_state() == EXITING) break ;
       }

// Get the result of the measurement from the sensor

       int distance = distanceSensor.getDistance(); 

// Print the value to the terminal

       printf("Distance(mm): %d\n", distance);

// Check to see if CTL-C hit

       if (rc_get_state() == EXITING) break ;

    } // end for loop

// We need to clean up after ourselves

    cpi2c_close(I2C_BUS) ;

// Remove the lock file

   rc_remove_pid_file();	

   return(0);

} // end main 

