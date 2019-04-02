#include "arcdefs.h"

// 
// Here are the routines in the arclib
//

// Dump variables to screen

void arc_var_dump(void) ;

// Print robot location

void arc_print_location(arc_location_t  loc) ;

// Routine to toggle the green LED 

void toggleGreenLED(void) ;

// Routine to convert degrees to radians

double arc_deg2rad(double deg) ;

// Routine to convert radians to degrees

double arc_rad2deg(double rad) ;

// Used to initialize the various modules and data structures we need

int  arc_init(void) ;

// Used to update robot's location in arena

void arc_update_location() ;

// Helper function

void  arc_move_init(int right_dir, int left_dir, bool soft_start) ;

// Routine to move the robot

void  arc_move(void) ;

// Routine to rotate the robot

void arc_rotate(double angle) ;

// Routine used to move forward

void  arc_forward(double distance, double velocity) ;

// Setup and cleanup routines

int  arc_setup(void) ;

int arc_config(void) ;

int arc_cleanup(void) ;

bool arc_go_to_goal(double x, double y);

bool arc_go_to_angle(double angle);

int  getBoxLocation();

int  getGreenLocation();

