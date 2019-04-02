// Need access to std i/o routines 

#include <stdio.h>

// Need access to i2c library funcitions

#include <robotcontrol.h>

// Need access to servo_driver.h

#include "servo_driver.h"

// Routine to set the PWM freqeuncy
// Routine accepts a freqeuncy between 40 Hz and 1000 Hz
// and sets the pre-scaler value in the PCA9685 servo controller

#define   DEBUG    0

unsigned int resetServoDriver(void) {

// Set the I2C device address

   rc_i2c_set_device_address(I2C_BUS, SERVO_I2C_ADDR) ;

// Write to MODE1 register

   rc_i2c_write_byte(I2C_BUS, PCA9685_MODE1, 0x00) ;

   return(TRUE);
}

// ******************************************************************************

unsigned int setServoFREQ(double freq) {
   double         tmp ;
   uint8_t        pre_scale_value ;
   uint8_t        old_mode ;
   uint8_t        new_mode ;

// Set up a read buffer

   unsigned char rd_buf[BUF_SIZE] ;

// Make sure frequency is in the allowed range (40 Hz to 1 kHz)
// If not, then return an error

   if ((freq < 40.0) || (freq > 1000)) return(FALSE) ;
 
// Convert freqeuncy to a pre-scaler value

   tmp = (25.0e6 / (4096.0 * freq)) + 0.5 ;
   pre_scale_value = ((unsigned char) tmp)  - 1 ;
   if (DEBUG) printf("Pre-scaler value: %d\n", (int) pre_scale_value) ;

// Select the device we want to talk to

   rc_i2c_set_device_address(I2C_BUS, SERVO_I2C_ADDR) ;

// Need to read current mode

   rc_i2c_read_byte(I2C_BUS, PCA9685_MODE1, rd_buf) ;
   old_mode = rd_buf[0] ;
   new_mode = (old_mode & 0x7f) | 0x10 ; // sleep mode

// Write out new mode and put PCA9685 to sleep

   rc_i2c_write_byte(I2C_BUS, PCA9685_MODE1, new_mode) ;

//  Write out the prescaler value

   rc_i2c_write_byte(I2C_BUS, PCA9685_PRESCALE, pre_scale_value) ;

// Restore old mode setting

   rc_i2c_write_byte(I2C_BUS, PCA9685_MODE1, old_mode) ;

// Wait 5 ms 

   rc_usleep(5000) ;

// Turn on auto increment

   rc_i2c_write_byte(I2C_BUS, PCA9685_MODE1, (old_mode | 0xa1) ) ;   

   return(TRUE);
}

// ******************************************************************
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// ******************************************************************

// Routine to set the pulse width in ticks
// 0.5 ms to 2ms is 100 to 400

unsigned int setServoPW(int chan, int pw) {
   uint8_t   ms_byte, ls_byte ;
   uint8_t   cr_addr ;

// Set up a write buffer

   uint8_t  wr_buf[BUF_SIZE] ;

// Make sure we have a valid channel number

   if ((chan < 0) || (chan > 15)) return(FALSE) ;

   if (DEBUG) printf("Channel #:  %d\n", chan) ;

// Select the device we want to talk to

   rc_i2c_set_device_address(I2C_BUS, SERVO_I2C_ADDR) ;

// Control register base address for a particular servo
// can be computed by taking 4 * channel number + 6

   cr_addr = (unsigned char) (4 * chan + SERVO_0_ON_L);
   if (DEBUG) printf("Control register address is %d\n", (int) cr_addr) ;

// Need to break the pulse with up into two bytes

   ms_byte = (unsigned char) (pw / 256) ;
   ls_byte = (unsigned char) (pw % 256 );
   if (DEBUG) printf("ms_byte = %d, ls_byte = %d\n", (int) ms_byte, (int) ls_byte) ;

// Base address for control register

   wr_buf[0] = cr_addr ;

// Pulse turned on when counter equals $000

   wr_buf[1] = 0x00 ;
   wr_buf[2] = 0x00 ;

// Pulse turned off when counter equals ...

   wr_buf[3] = ls_byte ;
   wr_buf[4] = ms_byte ;

   rc_i2c_write_bytes(I2C_BUS, wr_buf[0], 4, &wr_buf[1]) ;

   return(TRUE);
}
