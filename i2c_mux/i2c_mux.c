// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// The color sensor shares device address 0x29 with
// the distance sensors so the solution is to use
// the Sparkfun i2c mux (8 ports)
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//
// We need our custom i2c routines
//
#include <stdint.h> // for uint8_t types etc
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h> //for IOCTL defs

#include   <robotcontrol.h>
#include   "i2c_mux.h"

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to enable a mux port
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void enableMuxPort(uint8_t  portNumber) {
   uint8_t    settings ;
   int        fd, ret ;

// Select the MUX as the device we want to talk to

   rc_i2c_set_device_address(MUX_I2C_BUS, MUX_I2C_ADDR) ;

// Read the current MUX settings

   fd = rc_i2c_get_fd(MUX_I2C_BUS) ;
   ret = read(fd, &settings, 1) ;
   if (ret != 1) printf("read failure in enableMuxPort\n") ;

// Set the wanted bit to enable the port

   if (portNumber <= 7) {
      settings |= (1 << portNumber) ;
   } else {
      settings = 0xff ;
   }

// Write the byte to the MUX 
// No register address

   rc_i2c_send_byte(MUX_I2C_BUS, settings) ;

   return ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to disable a mux port
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 

void disableMuxPort(uint8_t  portNumber) {
   uint8_t    settings ;
   int        fd, ret ;

// Select the MUX as the device we want to talk to

   rc_i2c_set_device_address(MUX_I2C_BUS, MUX_I2C_ADDR) ;

// Read the current MUX settings

   fd = rc_i2c_get_fd(MUX_I2C_BUS) ;
   ret = read(fd, &settings, 1) ;
   if (ret != 1) printf("read failure in disableMuxPort\n") ;

// Clear the wanted bit to disable the port

   if (portNumber <= 7) {
       settings &= ~(1 << portNumber) ;
   } else {
       settings = 0x00 ;
   }

// Write the byte to the MUX 
// No register address

   rc_i2c_send_byte(MUX_I2C_BUS, settings) ;

   return ;
}


