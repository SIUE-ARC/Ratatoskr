//
// Package of servo related functions
//

#include  "servo_pkg.h"

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//  Routine to setup servo related stuff
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
int servo1 = 1500;
int servo2 = 1500;
void  servo_setup(void) {

// Turn of 6V servo power rails

   rc_servo_init() ;
   rc_servo_power_rail_en(ON) ;
// Set up thread to control Servo
   pthread_t  servo1_pthread;
   pthread_t  servo2_pthread;
   rc_pthread_create(&servo1_pthread,servo1_thread,(void*)&servo1,SCHED_FIFO,2);
   rc_pthread_create(&servo2_pthread,servo2_thread,(void*)&servo2,SCHED_FIFO,2);
   return ;

} // end servo_setup() ;

// servo1 is gripper
void *servo1_thread(void* data)
{
   int angle;
    
   while(1){
	   angle = *((int*)data);   	// cast void pointer into int pointer and dereference it
	   rc_servo_send_pulse_us(DISTANCE_SENSOR_SERVO_CHANNEL, angle) ;
	   rc_usleep(20000) ;
   }
}

//claw
void *servo2_thread(void* data)
{
	   int angle;
   while(1){
	   angle = *((int*)data);   	// cast void pointer into int pointer and dereference it
	   rc_servo_send_pulse_us(DISTANCE_SENSOR_SERVO_CHANNEL, angle) ;
	   rc_usleep(20000) ;
   }
}
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//  Routine to cleanup servo related stuff
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void servo_cleanup(void) {

   rc_servo_power_rail_en(OFF) ;
   rc_servo_cleanup() ;

   return ;

} // end servo_cleanup() 

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to command distance sensor servo
// to a specified angle
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void distance_sensor_angle(int channel, int angle) {


   while(1==1){
   rc_servo_send_pulse_us(DISTANCE_SENSOR_SERVO_CHANNEL, angle) ;
   rc_usleep(20000) ;
   }
   return ;

} // end distance_sensor_servo() 

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Routine to sweep the distance sensor 
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void sweep_distance_sensor(void) {
   int   angle, i ;

   servo_setup() ;

// We could initialize the distance sensor here


// Command servo to -90 degrees
// Wait a second

   //distance_sensor_angle(-60) ;
   rc_usleep(500000) ;

// Now sweep in 10 degree increments
// So like 18 steps
// About 200ms per step

   /*for (angle = -90; angle <= 90; angle += 10) {
        for (i = 1; i <= 10; i++) {
            distance_sensor_angle(angle) ;

// We could take some distance readings here

            rc_usleep(20000) ;
         }
   }
*/
// Command servo to 0 degrees
// Wait a bit

   rc_usleep(50000) ;
   //distance_sensor_angle(-90) ;
   rc_usleep(50000) ;
   servo_cleanup() ;
   return ;
} // end sweep_distance_sensor()
   






