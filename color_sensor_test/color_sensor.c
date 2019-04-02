// Need access to std i/o routines 

#include <stdio.h>

// Need access to i2c library funcitions

#include   <robotcontrol.h>

// Color sensor related defines

#include   "color_sensor.h"

// We need our i2c MUX routines

#include   "i2c_mux.h"

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to initialize the TCS3475 color sensor
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void init_color_sensor(void){

// Set up a read buffer

   uint8_t rd_buf[BUF_SIZE] ;

// Set up a write buffer

   uint8_t wr_buf[BUF_SIZE] ;

// Make sure that the color sensor is selected

   rc_i2c_set_device_address(COLOR_SENSOR_I2C_BUS, COLOR_SENSOR_ADDR) ;

// Need to enable the sensor by writing value to the ENABLE register

   wr_buf[0] = CMD_BIT | ENABLE ;

// Setting the lower two bits should enable the sensor

   wr_buf[1] = 0x03 ;
   rc_i2c_write_byte(COLOR_SENSOR_I2C_BUS, wr_buf[0], wr_buf[1]) ;

// Let's read the ID register 
// Print it to the screen when debugging
// Check it to make sure it reads 0x44

   wr_buf[0] = CMD_BIT | ID ;
   rc_i2c_read_byte(COLOR_SENSOR_I2C_BUS, wr_buf[0], rd_buf) ;
   
   if (COLOR_SENSOR_DEBUG) {
         printf("We expect 0x44 and the color sensor returned the ID: %x\n", rd_buf[0]) ;
   }

// Let's set the gain of the sensor

   wr_buf[0] = CMD_BIT | CONTROL ;
   wr_buf[1] = GAIN_16X ;
   rc_i2c_write_byte(COLOR_SENSOR_I2C_BUS, wr_buf[0], wr_buf[1]) ;
   if (COLOR_SENSOR_DEBUG) {
         printf("Gain setting is: %x\n", wr_buf[1]) ;
   }

// Let's set the integration time of the sensor

   wr_buf[0] = CMD_BIT | ATIME ;
   wr_buf[1] = INTEG_TIME ;
   rc_i2c_write_byte(COLOR_SENSOR_I2C_BUS, wr_buf[0], wr_buf[1]) ;
   if (COLOR_SENSOR_DEBUG) {
         printf("Integration time is: %x\n", (int) wr_buf[1]) ;
   }

   return ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to cleanup the TCS3475 color sensor
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void  cleanup_color_sensor(void) {
   unsigned char wr_buf[BUF_SIZE] ;

// Need to disable the sensor by writing value to the ENABLE register
// Clearing the lower 2 bits should disable the sensor

   wr_buf[0] = CMD_BIT | ENABLE ;
   wr_buf[1] = 0x00 ;
   rc_i2c_write_byte(COLOR_SENSOR_I2C_BUS, wr_buf[0], wr_buf[1]) ;

   return ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to read the TCS3475 color sensor
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void read_color_sensor(unsigned int *c, 
                       unsigned int *r, unsigned int *g, unsigned int *b) {

// Set up a read buffer

   uint8_t rd_buf[BUF_SIZE] ;

// Set up a write buffer

   uint8_t wr_buf[BUF_SIZE] ;

// Let's get the "clear" data from the sensor 

   wr_buf[0] = CMD_BIT | CDATA ;
   rc_i2c_read_bytes(COLOR_SENSOR_I2C_BUS, wr_buf[0], 2, rd_buf);
   *c = (unsigned int) rd_buf[0] ;
   *c |= ((unsigned int) rd_buf[1]) << 8 ;

// Let's get the "red" data from the sensor 

   wr_buf[0] = CMD_BIT | RDATA ;
   rc_i2c_read_bytes(COLOR_SENSOR_I2C_BUS, wr_buf[0], 2, rd_buf);
   *r = (unsigned int) rd_buf[0] ;
   *r |= ((unsigned int) rd_buf[1]) << 8 ;

// Let's get the "green" data from the sensor 

   wr_buf[0] = CMD_BIT | GDATA ;
   rc_i2c_read_bytes(COLOR_SENSOR_I2C_BUS, wr_buf[0], 2, rd_buf);
   *g = (unsigned int) rd_buf[0] ;
   *g |= ((unsigned int) rd_buf[1]) << 8 ;

// Let's get the "blue" data from the sensor 

   wr_buf[0] = CMD_BIT | BDATA ;
   rc_i2c_read_bytes(COLOR_SENSOR_I2C_BUS, wr_buf[0], 2, rd_buf);
   *b = (unsigned int) rd_buf[0] ;
   *b |= ((unsigned int) rd_buf[1]) << 8 ;

   return ;
}


