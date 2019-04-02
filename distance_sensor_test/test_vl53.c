/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/

#include    <stdio.h>
#include    <stdint.h>
#include    "VL53L1X.h"
#include    "Arduino.h"
#include    <robotcontrol.h>

#include    "i2c_mux.h"

// I2C MUX port number for the 3 distance sensors

#define     FRONT    0
#define     RIGHT    5
#define     LEFT     2

// i2c address for distance sensors

#define    DIST_I2C_ADDR    0x29

// Our sensor objects

VL53L1X front_sensor ;
VL53L1X left_sensor ;
VL53L1X right_sensor ;

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// General setup
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void general_setup()
{
// Check for existing processes

    if (rc_kill_existing_process(2.0)<-2) {
        printf("\n") ;
    }

// Create a lock file

    rc_make_pid_file() ;

// Start signal handler so we can exit cleanly

    if(rc_enable_signal_handler()==-1) {
        fprintf(stderr,"ERROR: failed to start signal handler\n");
    }

// Set the state to running

    rc_set_state(RUNNING) ;

// Initialize the i2c bus

   rc_i2c_init(I2C_BUS, MUX_I2C_ADDR) ;

}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Front sensor setup
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void front_sensor_setup(void) {

// Set i2c mux to FRONT port

  rc_i2c_set_device_address(I2C_BUS, MUX_I2C_ADDR) ;
  disableMuxPort(ALL_PORTS) ;
  enableMuxPort(FRONT) ;

// Now set i2c address to 0x29

  rc_i2c_set_device_address(I2C_BUS, DIST_I2C_ADDR) ;

// Sensor config
  
  front_sensor.setTimeout(500);
  if (!front_sensor.init()) {
     printf("Failed to detect and initialize sensor!");
     while (1) {
         if (rc_get_state() == EXITING) break ;
     }
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.

  front_sensor.setDistanceMode(VL53L1X::Long);
  front_sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.

  front_sensor.startContinuous(50);
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Left sensor setup
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void left_sensor_setup(void) {

// Set i2c mux to LEFT port

  rc_i2c_set_device_address(I2C_BUS, MUX_I2C_ADDR) ;
  disableMuxPort(ALL_PORTS) ;
  enableMuxPort(LEFT) ;

// Now set i2c address to 0x29

  rc_i2c_set_device_address(I2C_BUS, DIST_I2C_ADDR) ;

// Sensor config
  
  left_sensor.setTimeout(500);
  if (!left_sensor.init()) {
     printf("Failed to detect and initialize sensor!");
     while (1) {
         if (rc_get_state() == EXITING) break ;
     }
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.

  left_sensor.setDistanceMode(VL53L1X::Long);
  left_sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.

  left_sensor.startContinuous(50);
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Right sensor setup
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void right_sensor_setup(void) {

// Set i2c mux to RIGHT port

  rc_i2c_set_device_address(I2C_BUS, MUX_I2C_ADDR) ;
  disableMuxPort(ALL_PORTS) ;
  enableMuxPort(RIGHT) ;

// Now set i2c address to 0x29

  rc_i2c_set_device_address(I2C_BUS, DIST_I2C_ADDR) ;

// Sensor config
  
  right_sensor.setTimeout(500);
  if (!right_sensor.init()) {
     printf("Failed to detect and initialize sensor!");
     while (1) {
         if (rc_get_state() == EXITING) break ;
     }
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.

  right_sensor.setDistanceMode(VL53L1X::Long);
  right_sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.

  right_sensor.startContinuous(50);
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//   MAIN PROGRAM
//
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
int main(void) {

  double  inches ;
    
// Call the setup routine

  general_setup() ;

// Sensor setup 

  front_sensor_setup() ;
  right_sensor_setup() ;
  left_sensor_setup() ;

// Set i2c mux to RIGHT port

  rc_i2c_set_device_address(I2C_BUS, MUX_I2C_ADDR) ;
  disableMuxPort(ALL_PORTS) ;
  enableMuxPort(RIGHT) ;
  rc_i2c_set_device_address(I2C_BUS, DIST_I2C_ADDR) ;

// Sit in an infinite loop and take readings

   while (1) {
       if (rc_get_state() == EXITING) break ;
       inches = 0.03937 * right_sensor.read() ;
       printf("Right sensor reading: \t%f\r", inches);
       if (right_sensor.timeoutOccurred()) { 
            printf("TIMEOUT\n"); 
       }
    }

// We need to clean up after ourselves

    rc_i2c_close(I2C_BUS) ;

// Remove the lock file

   rc_remove_pid_file();	

   return(0);

} // end main 
