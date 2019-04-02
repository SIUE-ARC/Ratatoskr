#include   "i2c_mux.h"
#include   "VL53L1X.h"
#include   "Arduino.h"


// I2C MUX port number for the 3 distance sensors

#define     FRONT    0
#define     RIGHT    5
#define     LEFT     2

// i2c address for distance sensors

#define    DIST_I2C_ADDR    0x29

extern VL53L1X front_sensor ;
extern VL53L1X left_sensor ;
extern VL53L1X right_sensor ;

void front_sensor_setup(void) ;
void left_sensor_setup(void) ;
void right_sensor_setup(void) ;
double distance_sensor(void) ;

