//
// Servo package
//

#include    <stdio.h>
#include    <getopt.h>
#include    <stdlib.h>
#include    <robotcontrol.h>

// Servo channel number

#define   DISTANCE_SENSOR_SERVO_CHANNEL    1

// Slope is in usec / degree

#define   DISTANCE_SERVO_SLOPE             10
#define   DISTANCE_SERVO_OFFSET            1500     

#define   ON     1
#define   OFF    0     


void *servo1_thread(void*);
void *servo2_thread(void*);
void  servo_setup(void) ;
void  servo_cleanup(void) ;
void  distance_sensor_angle(int channel,int angle) ;
void  sweep_distance_sensor(void) ;


