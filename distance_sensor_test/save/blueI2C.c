/* 

   blueI2C: BeagleBoneBlue implementation of cross-platform I^C routines

   This file is part of CrossPlatformI2C.

   CrossPlatformI2C is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   CrossPlatformI2C is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with CrossPlatformI2C.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdint.h> // for uint8_t types etc
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h> //for IOCTL defs

#include "CrossPlatformI2C.h"
#include <robotcontrol.h>

#define   DEBUG      0
#define   I2C_BUS    1

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Distance sensor uses this!!!!
//
// Write multiple (specifiec by count) bytes to register but winkle is that the register
// address is two bytes wide.
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

bool  cpi2c_writeRegisters(uint8_t address, uint16_t subAddress, uint8_t count, uint8_t *data) 
{
  uint8_t   buf[2] ;
  int       ret ;

// Set the device address

  rc_i2c_set_device_address(I2C_BUS, address) ;

 // Get MS and LS bytes bytes

   buf[1] = (uint8_t) (subAddress >> 8 );
   buf[0] = (uint8_t) (subAddress & 0xFF) ;

// Send the 2 byte register address

   ret = rc_i2c_send_bytes(I2C_BUS, 2, buf) ;
   if (ret == -1) {
      printf("rc_i2c_send_bytes (register) failed in cpi2c_writeMultipleRegisters_16\n") ;
      return false ;
   }

// Write multiple bytes

   ret = rc_i2c_send_bytes(I2C_BUS, count, data) ;
   if (ret == -1) {
      printf("rc_i2c_send_bytes failed in cpi2c_writeMultipleRegisters_16\n") ;
      return false ;
   }
   
  return true ;

}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to open up an I2C bus 
// This is used by distance sensor.
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uint8_t cpi2c_open(uint8_t address, uint8_t bus)
{
    int   ret ;

    if (DEBUG) {
        printf("In cpi2c_open().\n") ;
        fflush(stdout) ;
    }

// This robotocontrol API call should be equivalent

    ret = rc_i2c_init(bus, address) ;
    if (ret == -1) printf("rc_i2c_init in cpi2c_open failed\n") ;
    return address ;

/*
    (void)bus;
    return address;
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to close the I2C connection to the device
// Doesn't look like distance sensor uses this
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void cpi2c_close(uint8_t device)
{
    int ret ;

    ret = rc_i2c_close(device) ;
    if (ret == -1) printf("rc_i2c_close in cpi2c_close failed\n") ;
    return ;

/*
    (void)device;
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to write a single byte to a register in device
// Distance sensor uses this!
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

bool cpi2c_writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
   int  ret ;

// Set the device address

   rc_i2c_set_device_address(I2C_BUS, address) ;

// Write a single byte to register

   ret = rc_i2c_write_byte(I2C_BUS, subAddress, data) ;
   if (ret == -1) {
       printf("rc_i2c_write in cpi2c_writeRegister failed\n") ;
       return false ;
   } else {
        return true ;
   }

/*
    Wire.beginTransmission(address);    // Initialize the Tx buffer
    Wire.write(subAddress);             // Put slave register address in Tx buffer
    Wire.write(data);                   // Put data in Tx buffer
    return Wire.endTransmission() == 0; // Send the Tx buffer
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to read a register
// The winkle here is that the subAddress is 2 bytes long!
// Distance sensor uses this!
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uint8_t cpi2c_readRegister(uint8_t address, uint16_t subAddress)
{
   uint8_t   data ;
   uint8_t   buf[2] ;
   int       ret ;
   int       fd ;

// Set the device address

   rc_i2c_set_device_address(I2C_BUS, address) ;

// Get MS and LS bytes bytes

   buf[0] = (uint8_t) (subAddress >> 8) ;
   buf[1] = (uint8_t) (subAddress & 0xFF) ;

// Send the 2 byte register address

   ret = rc_i2c_send_bytes(I2C_BUS, 2, buf) ;
   if (ret == -1) {
      printf("rc_i2c_send_bytes failed in cpi2c_readRegister\n") ;
      fflush(stdout) ;
      return 0 ;
   }

// Read a single byte from the register
// We will need to get the file identifier for the bus

   fd = rc_i2c_get_fd(I2C_BUS) ;
   ret = read(fd, &data, 1);
   if (ret !=1 ) {
      printf("read in cpi2c_readRegiser failed\n") ;
      return 0 ;
   }
   
   return data ;

/*
    Wire.beginTransmission(address);
    Wire.write(subAddress >> 8); //MSB
    Wire.write(subAddress & 0xFF); //LSB
    if (Wire.endTransmission() != 0) //Send a restart command. Do not release bus.
        return 0; //Sensor did not ACK

    Wire.requestFrom((uint8_t)address, (uint8_t)1);
    if (Wire.available())
        return (Wire.read());
    
    return 0; //Error: Sensor did not respond
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to read multiple bytes (number specified by count)
// Doesn't look like distance sensor uses this!
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void cpi2c_readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{ 
   int   ret ;

// Set the device address

   rc_i2c_set_device_address(I2C_BUS, address) ;

// Read the specified number of bytes beginning at sub-address

   ret = rc_i2c_read_bytes(I2C_BUS, subAddress, count, dest) ;
   if (ret == -1) printf("rc_i2c_read_bytes failure in cpi2c_readRegisters\n") ;

   return ;

/* 
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);      // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); 
    } 
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to read a word
// Doesn't look like distance sensor uses this!
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


uint16_t cpi2c_readRegister_8_16(uint8_t address, uint8_t subAddress)
{

  uint16_t   data ;
  int        ret ;

// Set device address

  rc_i2c_set_device_address(I2C_BUS, address) ;

// Read a word from the specified register address

  ret = rc_i2c_read_word(I2C_BUS, subAddress, &data) ;
  if (ret == -1) printf("rc_i2c_read failure in cpi2c_readRegister_8_16\n") ;

// Return the word that we read

  return data ;

/*
    Wire.beginTransmission(address);
    Wire.write(subAddress); 
    if (Wire.endTransmission() != 0) //Send a restart command. Do not release bus.
        return 0; //Sensor did not ACK

    Wire.requestFrom((uint8_t)address, (uint8_t)2);
    if (Wire.available())
    {
        uint8_t lsb = Wire.read();
        uint8_t msb = Wire.read();
        return ((uint16_t)msb << 8 | lsb);
    }
    
    return 0; //Error: Sensor did not respond
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to read a register (word length)
// The winkle here is that the subAddress is 2 bytes long!
// Distance sensor uses this!!!
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uint16_t cpi2c_readRegister_16(uint8_t address, uint16_t subAddress)
{
  uint8_t   buf[2] ;
  uint16_t  value ;
  int       ret ;
  int       fd ;

// Set the device address

   rc_i2c_set_device_address(I2C_BUS, address) ;

// Get MS and LS bytes bytes

   buf[0] =  (uint8_t) (subAddress >> 8) ;
   buf[1] =  (uint8_t) (subAddress & 0xFF) ;

/*
   if (DEBUG) {
     printf("cpi2c_readRegister_16: subAddress = %x, buf[0] = %u, buf[1] = %u \n", subAddress, buf[0], buf[1]) ;
     fflush(stdout);
    }
*/

// Send the 2 byte register address

   ret = rc_i2c_send_bytes(I2C_BUS, 2, buf) ;
   if (ret == -1) {
      printf("rc_i2c_send_bytes failed in cpi2c_readRegister_16\n") ;
      return 0 ;
   }

// Read a word (2 bytes) from the register
// We will need to get the file identifier for the bus

   fd = rc_i2c_get_fd(I2C_BUS) ;
   ret = read(fd, buf, 2);
   if (ret != 2 ) {
      printf("read in cpi2c_readRegiser_16 failled\n") ;
      return 0 ;
   }

/*
   if (DEBUG) {
     printf("buf[0] = %u, buf[1] = %u \n", buf[0], buf[1]) ;
     fflush(stdout);
   }

*/
   value = ((uint16_t) buf[0] << 8) | ((uint16_t) buf[1]) ;

   return value ;

/*
    Wire.beginTransmission(address);
    Wire.write(subAddress >> 8); //MSB
    Wire.write(subAddress & 0xFF); //LSB
    if (Wire.endTransmission() != 0) //Send a restart command. Do not release bus.
        return 0; //Sensor did not ACK

    Wire.requestFrom((uint8_t)address, (uint8_t)2);
    if (Wire.available())
    {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        return ((uint16_t)msb << 8 | lsb);
    }
    
    return 0; //Error: Sensor did not respond
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Distance sensor uses this!!!!
// Write a single byte to register
// But the address for the register is 2 bytes long
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

bool cpi2c_writeRegister_16_8(uint8_t address, uint16_t subAddress, uint8_t data)
{

  uint8_t   buf[2] ;
  int       ret ;

// Set the device address

  rc_i2c_set_device_address(I2C_BUS, address) ;

 // Get MS and LS bytes bytes

   buf[0] = (uint8_t) (subAddress >> 8);
   buf[1] = (uint8_t) (subAddress & 0xFF) ;

// Send the 2 byte register address

   ret = rc_i2c_send_bytes(I2C_BUS, 2, buf) ;
   if (ret == -1) {
      printf("rc_i2c_send_bytes (register) failed in cpi2c_readRegister_16_8\n") ;
      return 0 ;
   }

// Write a single byte to the register

   ret = rc_i2c_send_byte(I2C_BUS, data) ;
   if (ret == -1) {
      printf("rc_i2c_send_byte (data) failed in cpi2c_readRegister_16_8\n") ;
      return 0 ;
   }
   
   return true ;
/*   
    Wire.beginTransmission(address);
    Wire.write(subAddress >> 8); //MSB
    Wire.write(subAddress & 0xFF); //LSB
    Wire.write(data);
    if (Wire.endTransmission() != 0)
        return false; //Error: Sensor did not ACK
  
    return true; // success
*/
}


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Distance sensor uses this!!!!
//
// Write a word to register but winkle is that the register
// address is two bytes wide.
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

bool cpi2c_writeRegister_16_16(uint8_t address, uint16_t subAddress, uint16_t data)
{
  uint8_t   buf[2] ;
  int       ret ;

// Set the device address

  rc_i2c_set_device_address(I2C_BUS, address) ;

 // Get MS and LS bytes bytes

   buf[1] = (uint8_t) (subAddress >> 8) ;
   buf[0] = (uint8_t) (subAddress & 0xFF) ;

// Send the 2 byte register address

   ret = rc_i2c_send_bytes(I2C_BUS, 2, buf) ;
   if (ret == -1) {
      printf("rc_i2c_send_bytes (register) failed in cpi2c_readRegister_16_16\n") ;
      return false ;
   }

// Take the word and split it into two bytes

   buf[1] = (uint8_t) (data >> 8) ;
   buf[0] = (uint8_t) (data & 0xFF) ;

// Write two bytes to the register

   ret = rc_i2c_send_bytes(I2C_BUS, 2, buf) ;
   if (ret == -1) {
      printf("rc_i2c_send_bytes (data word) failed in cpi2c_writeRegister_16_16\n") ;
      return false ;
   }
   
  return true ;

/*
    Wire.beginTransmission(address);
    Wire.write(subAddress >> 8); //MSB
    Wire.write(subAddress & 0xFF); //LSB
    Wire.write(data >> 8); //MSB
    Wire.write(data & 0xFF); //LSB
    if (Wire.endTransmission() != 0)
        return 0; //Error: Sensor did not ACK
    
    return true; // success
*/
}




// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Distance sensor uses this!!!!
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


void cpi2c_beginTransmission(uint8_t address)
{

// Set the device address

  rc_i2c_set_device_address(I2C_BUS, address) ;
  return ;

/*
    Wire.beginTransmission(address);
*/
}


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Distance sensor uses this!!!!
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uint8_t cpi2c_write(uint8_t data)
{
   int  ret ;

// Following command should be equivalent

  ret = rc_i2c_send_byte(I2C_BUS, data) ;
  if (ret == -1) printf("rc_i2c_send_byte failed in cpi2c_write\n") ;
  return (uint8_t) ret ;

/*
    return Wire.write(data);
*/
}


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Doesn't look like distance sensor uses this!!!!
// Just return 1
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uint8_t cpi2c_read(void)
{
   return 1 ;
/*
    return Wire.read();
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Distance sensor uses this!!!
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uint8_t cpi2c_endTransmission(bool stop)
{
   (void) stop ;
   return 1 ;

/*
    return Wire.endTransmission(stop);
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Doesn't look like distance sensor uses this!!!!
// Just return 1
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uint8_t  cpi2c_requestFrom(uint8_t address, uint8_t count)
{
   (void) count ;
   rc_i2c_set_device_address(I2C_BUS, address) ;
   return 1 ;
/*
    return Wire.requestFrom(address, count);
*/
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Doesn't look like distance sensor uses this!!!!
// Just return 0
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uint8_t  cpi2c_available(void)
{
   return 1 ;
/*
    return Wire.available();
*/
}



