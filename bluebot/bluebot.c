//
// @file    bluebot.c
//
// Simple robot program using beaglebone blue
// Revised on March 14, 2019
// Demonstrate working I2C mux which had a color
// sensor along with 3 distance sensors connected to it.
//

#include    <stdio.h>
#include    <stdlib.h>
#include    <signal.h>
#include    <getopt.h>
#include    <math.h>
#include    <robotcontrol.h>

#include    "arclib.h"
#include    "VL53L1X.h"
#include    "distance_sensor_pkg.h"
#include    "servo_pkg.h"

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Create our robot structure as a global
// That way all our arc routines will have easy 
// access to it 
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

arc_robot_t     robot ;
bool            DEBUG = false ;

rc_filter_t     left_motor_filter = RC_FILTER_INITIALIZER ;
rc_filter_t     right_motor_filter = RC_FILTER_INITIALIZER ;

// Distance sensors

VL53L1X front_sensor ;
VL53L1X left_sensor ;
VL53L1X right_sensor ;

#define GRIPPER_OPEN 1550
#define GRIPPER_CLOSED 900
#define ARM_DOWN 2100
#define ARM_UP 900
extern int servo1;
extern int servo2; // extern  means this variable this variable is declared in another file and is used in this file 

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// MAIN PROGRAM
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

int  main() {
  double   distance ;

// Initiilization

   arc_setup() ;
   arc_config() ;
   arc_init() ;
	
   servo_setup();

// Test distance sensor servo

    //sweep_distance_sensor();

// Test forward looking distance sensor
   
   //distance = distance_sensor() ;
   //printf("Front sensor reading: \t%f\n", distance);
   //getBoxLocation();
  // fflush(stdout) ;
   //arc_rotate(arc_deg2rad(-90.0)) ;
   //arc_forward(15, 6.0) ; 
  //arc_go_to_goal(5,5);
	char command;
	//arc_forward(10,6);
   	while(true)
   	{
		printf("type command g=gotogoal, f = go forward, p = print, c = close a = angle task1 = 1 task2 = 2 \n");
		command = getchar();
		if( command == 'g'){
			double x;
			double y;
			printf("Enter first number: ");
			scanf("%lf", &x);
			printf("Enter first number: ");
			scanf("%lf", &y);
			arc_go_to_goal(x,y);
		        arc_print_location(robot.location);	
		}
		if( command == 'a') {
			double d;
			printf("Enter first number: ");
			scanf("%lf", &d);
			arc_go_to_angle(d);		
		}
		if( command == 's') {
			int d;
			printf("Enter channel : ");
			scanf("%d", &d);
			servo1 = d; 
		}
		if( command == 'f') {
			double d;
			printf("Enter first number: ");
			scanf("%lf", &d);
			arc_forward(d,6);		
		}
		if( command == 'p') {
			arc_print_location(robot.location);		
		}
		if( command == 'd') {
			distance = distance_sensor() ;
  		 	printf("Front sensor reading: \t%f\n", distance);
		}

		if( command == '1') {
			double x;
			double y;
			printf("Enter first number: ");
			scanf("%lf", &x);
			printf("Enter first number: ");
			scanf("%lf", &y);
			arc_go_to_goal(x-12,y-12);
			double deltaX,deltaY,beta,currentX,currentY;
			currentX = robot.location.xy.d[X];
			currentY = robot.location.xy.d[Y];
			deltaX = x-currentX;
			deltaY = y-currentY;
			printf("Robot going to (%f,%f)",deltaX,deltaY);
			beta = atan2(deltaY,deltaX);
			printf("Robot is rotating to face box");
			arc_go_to_angle(beta);	
		}
		if( command == '2') {
			int cameraX;
			int goalX = 150;
			int tolerance = 15;
			//arc_rotate(M_PI);
			//if camera sees a box and returns
			//while(true) {
				cameraX = getBoxLocation();
				/*if(cameraX >-100000) {
					// if camera sees a box outside of tolerence towards -
					if(cameraX < 150-tolerance)
					{
						//arc_go_to_angle(robot.location.theta+10);
					}
					// if camera sees a box outside of tolerence towards +
					else if(cameraX > 150+tolerance)
					{
						//arc_go_to_angle(robot.location.theta-10);
					}
					// if camera sees a box inside of tolerance 
					else
					{
						break;
					}
			
				}*/
			//}
				
		}
	
		if( command == '3') {
			arc_rotate(M_PI-.122);
			distance = distance_sensor() ;
  		 	printf("Front sensor reading: \t%f\n", distance);
			if(distance < 18)
			{
				arc_forward(distance -3,6);
				distance = distance_sensor() ;
				printf("Front sensor reading: \t%f\n", distance);
				if(distance >1.8)
				arc_forward(distance -1,6);
  		 		
			}
			
		}
	
		if( command == 'c') {
			break;		
		}
	
		/*if( command == '') {
			printf("no command \n");
		}*/
	}
 
  // distance = distance_sensor() ;
  // printf("Front sensor reading: \t%f\n", distance);
  // fflush(stdout) ;
  // arc_forward(distance / 2.0, 6.0) ; 

  // distance = distance_sensor() ;
  // printf("Front sensor reading: \t%f\n", distance);
  // fflush(stdout) ;
  // arc_forward(distance / 2.0, 6.0) ; 

// Demo rotate
	
   //arc_rotate(arc_deg2rad(-90.0)) ;
   //arc_rotate(arc_deg2rad(90.0)) ; 

   arc_print_location(robot.location);
// General cleanup

   arc_cleanup() ;

   return 0 ;

} // end main()
