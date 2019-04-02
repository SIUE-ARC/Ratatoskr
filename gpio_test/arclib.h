#include "arcdefs.h"

// 
// Here are the routines in the arclib
//

// Used to set configuration data for robot

int            arc_config(void) ;

// Used to initialize the various modules and data structures we need

int            arc_init(arc_location_t loc) ;

// Used to update robot's location in arena

arc_location_t arc_update_location(arc_location_t  cur_loc, int ticsR, int ticsL) ;

// Used to move robot to a new location 

int            arc_goto(arc_location_t loc) ;

// Used to cleanup before we exit program.

int            arc_cleanup(void) ;

// Dump variables to screen

void arc_var_dump(void) ;

// Print robot location

void arc_print_location(arc_location_t  loc) ;




