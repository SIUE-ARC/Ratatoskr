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
#include    "distance_sensor_pkg.h"

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Front sensor setup
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void front_sensor_setup(void) {

// Set i2c mux to FRONT port

  /*rc_i2c_set_device_address(I2C_BUS, MUX_I2C_ADDR) ;
  disableMuxPort(ALL_PORTS) ;
  enableMuxPort(FRONT) ; */
  

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

  front_sensor.setDistanceMode(VL53L1X::Short);
  front_sensor.setMeasurementTimingBudget(20000);

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

  right_sensor.setDistanceMode(VL53L1X::Long);   // Different sensor modes can be changed Short|Medium|Long
  right_sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.

  right_sensor.startContinuous(50);
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//  Test the distance sensor
//
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
double distance_sensor(void) {
   double  inches ;
    
// I2C bus will get initialized

  rc_i2c_init(MUX_I2C_BUS, DIST_I2C_ADDR) ;

// Sensor setup 

   front_sensor_setup() ;
//  right_sensor_setup() ;
// left_sensor_setup() ;

// Set i2c mux to RIGHT port

  /*rc_i2c_set_device_address(I2C_BUS, MUX_I2C_ADDR) ;
  disableMuxPort(ALL_PORTS) ;
  enableMuxPort(FRONT) ;
  */
//  We want talk to the distance sensor

   rc_i2c_set_device_address(I2C_BUS, DIST_I2C_ADDR) ;

   int i ;
   i = 0 ;
   while(1)   {
       if (rc_get_state() == EXITING) break ;
       inches = 0.03937 * right_sensor.read() ;

       i += 1 ;
       if (front_sensor.timeoutOccurred()) printf("TIMEOUT\n"); 
       if (i == 5) break ;
    }
   front_sensor.stopContinuous();

// Disable all the MUX ports

   //rc_i2c_set_device_address(MUX_I2C_BUS, MUX_I2C_ADDR) ;
   //disableMuxPort(ALL_PORTS) ;

// Close the i2c channel

   rc_i2c_close(MUX_I2C_BUS) ;	

   return inches ;

} // end main 
