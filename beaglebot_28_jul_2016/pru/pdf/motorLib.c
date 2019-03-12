//
// Library of routines which run on PRU0
// for handling the motors
//

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "mem.h"
#include "fix.h"
#include "motorLib.h"

// Pointer to shared memory is a global variable

  extern   shared_memory_t   *mem ;

// Wait for interrupt

  void    waitForInterrupt(void) ;

// *********************************************************
// Routine to halt PRU #1
// Sets halt flag bit in state variable
// We also have to clear the run flag.
// *********************************************************

void  haltPRU(void) {
   mem->command.status = ACTIVE ;
   mem->state = M_HALT ;
   return ;
}

// *********************************************************
// Routine to apply hard brake
// *********************************************************

void hardBrake(void) {
   mem->command.status = ACTIVE ;
   mem->state = M_HARD_BRAKE ;
   return ;
}

// **********************************************************
// Routine to coast to a stop
// **********************************************************

void coast(void) {
   mem->command.status = ACTIVE ;
   mem->state = M_COAST ;
   return ;
}
	
// ************************************************************
// Routine to implement PID loop on a single DC motor
// Using the velocity or differential PID
//
// delta_p = Kp * error
// delta_i = Ki * (error - past_error)
// delta_d = Kd * (error - past_error + 2 * past_past_error)
//
// output = previous_output + (delta_p + delta_i + delta_d)
//
// Errors in Q0 format.
// enc in Q0 format.
// Kp, Ki, Kd are in Q12 format,
// delta_p, delta_i, and delta_d are in Q format
// Output in Q0 format.
//
// **************************************************************

int32_t PID(DCmotor_t * motor, int32_t enc) {
   int32_t   delta_p, delta_i, delta_d ;
   int32_t   scr ;
   int32_t   out ;

// Update past_error and past_past_error
  
   motor->e2 = motor->e1 ;
   motor->e1 = motor->e0 ;

// Compute new error term

   motor->e0 = FSUB(motor->setpoint, enc) ;

// Compute delta_p, delta_i, and delta_d

   delta_p = FMUL(motor->Kp, motor->e0, 0) ;
   scr = FSUB(motor->e0, motor->e1) ;
   delta_i = FMUL(motor->Ki, scr, 0) ;
   scr = FADD(scr, motor->e2 << 1) ;
   delta_d = FMUL(motor->Kd, scr, 0) ;
   scr = FADD(delta_i, delta_d) ;
   scr = FADD(scr, delta_p) ;

// Convert the delta from Q to 0 format

   scr = FCONV(delta_p, Q, 0) ;
   out = FADD(motor->PWMout, scr)  ;

// Make sure "out" is in range

   if (out > motor->PWMmax) {
       out = motor->PWMmax ;
   } else {
       if (out < motor->PWMmin) {
           out = motor->PWMmin ;
       } // end if
   } // end if-then-else

// Save the output and also return it

   motor->PWMout = out ;   
   return   out ;
}

// ******************************************************************
// Routine to move.
// *********************************************************************

void move(void) { 
   int        i ;
   DCmotor_t  *motor ;
   int32_t    state ;

// Status is ACTIVE

   mem->command.status = ACTIVE ;

// Set the errors to zero
// Also set the distance traveled to 0
// The setpoint, brake mode, wheel direction
// and target distance all get set by the routines
// in robotLib. Slso clear out pwm array.
// enc array is cleared in PRU 1 asm code

   for (i=0; i<NUM_MOTORS; i++) {
      mem->motor[i].e0 = 0 ;
      mem->motor[i].e1 = 0 ;
      mem->motor[i].e2 = 0 ;
      mem->motor[i].distance = 0 ;
      mem->motor[i].PWMout = 0 ;
      mem->pwm[i] = 0 ;
   }

// Look at direction field so we can set the state correctly
// Also look at the breaking mode so when we stop
// we do so either by braking hard or by coasting.

   state = 0 ;

   if (mem->motorENA[M1]) {
        if (mem->motor[M1].wheelDirection == CW) {
           state |= M1_CW ;
        } else {
           state |= M1_CCW ;
        } // end if-then-else
        if (mem->motor[M1].brakeType == HARD) {
           state |= M_HARD_BRAKE ;
        } // end if
   } // end if

   if (mem->motorENA[M2]) {
        if (mem->motor[M2].wheelDirection == CW) {
           state |= M2_CW ;
        } else {
           state |= M2_CCW ;
        } // end if-then-else
        if (mem->motor[M2].brakeType == HARD) {
           state |= M_HARD_BRAKE ;
        } // end if
   } // end if

   if (mem->motorENA[M3]) {
        if (mem->motor[M3].wheelDirection == CW) {
           state |= M3_CW ;
        } else {
           state |= M3_CCW ;
        } // end if-then-else
        if (mem->motor[M3].brakeType == HARD) {
           state |= M_HARD_BRAKE ;
        } // end if
   } // end if
   if (mem->motorENA[M4]) {
        if (mem->motor[M4].wheelDirection == CW) {
           state |= M4_CW ;
        } else {
           state |= M4_CCW ;
        } // end if-then-else
        if (mem->motor[M4].brakeType == HARD) {
           state |= M_HARD_BRAKE ;
        } // end if
   } // end if

// Set the run bit
       
   state |= M_RUN ;

// Write state out to shared memory
// PRU1 should start generating PWM outputs

   mem->state = state ;

// Main loop
// Keep looping until distance traveled on a single
// motor exceeds the target distance.
// PRU 1 will interrupt us at the desired sample rate.
// The waitForInterrupt routine toggles the PRU LED
// at the sample rate to provide user with visual feedback.
//
 
   int32_t  scr ;
   int   loop = TRUE ;
   while (loop) {
      waitForInterrupt() ;
      for (i=0; i < NUM_MOTORS; i++) {
         if (mem->motorENA[i]) { 
            motor = &mem->motor[i] ;
            motor->distance = motor->distance + mem->enc[i] ;
            scr = motor->setpoint + motor->deltaSetpoint ;
            if (scr <= motor->targetSetpoint) {
                motor->setpoint = scr ;
            }
            if ((motor->distance) < (motor->targetDistance)) {
                mem->pwm[i] = PID(motor, mem->enc[i]) ;
            } else {
                loop = FALSE ;
                mem->state = M_STOP & mem->state ; // clear run bit  
            } // end if-then-else      
         } // end if
      } // end for
   } // end while
        
   return  ;

} // end move()

// *******************************************************************
//  Executes the command
//  The move() routine will look at the state and call appropriate
//  subroutine.  Upon return from the called routine, the status
//  will be updated to reflect that the commmand has COMPLETED.
//  The move() routine looks at the mem->motor struct to figure
//  out exactly what is must do.
// ******************************************************************	
	
void doCommand(int32_t  command_code) {

   switch (command_code) {
       case  NOP:        mem->command.status = IDLE ;                                 
                         break ;

       case  FWD:        move() ;
                         mem->command.status = COMPLETED ; 
                         break ;

       case  BWD:        move() ;
                         mem->command.status = COMPLETED ; 
                         break ;

       case  ROT:        move() ;
                         mem->command.status = COMPLETED ; 
                         break ;

       case BRAKE_HARD:  hardBrake() ;
                         mem->command.status = COMPLETED ; 
                         break ;

       case BRAKE_COAST: coast() ;
                         mem->command.status = COMPLETED ; 
                         break ;

       case HALT_PRU:    haltPRU() ;
                         mem->command.status = COMPLETED ; 
                         break ;
       
       default:          mem->command.status = IDLE ;          
                         break ;

   } // end switch

  return ;

} // end doCommand()
	
