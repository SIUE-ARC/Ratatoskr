//
// @file    blue_test.c
//
// Simple robot program using beaglebone blue
//

#include    <stdio.h>
#include    <stdint.h>
#include    <robotcontrol.h>

#define     X   0
#define     Y   1

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// MAIN PROGRAM
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

int  main() {
  uint64_t    a, b, c ;
  uint64_t    Ts_in_nanos ;
  double      Ts ;
  rc_vector_t cur_loc, tgt_loc ;

// Testing out timing functions

  Ts = 1.0 / 20 ;
  Ts_in_nanos = (uint64_t) (1e9 * Ts) ;
  printf("Ts in nanos is %ld\n", (long int) Ts_in_nanos) ;

  a = rc_nanos_since_boot() ;
  rc_usleep(25000) ;
  b = rc_nanos_since_boot() ;

  c = Ts_in_nanos - (b - a) ;
  c /= 1000 ;

  printf("Time difference is %ld usec.\n", (long int) c) ;

// Testing out vector functions
  
  cur_loc = RC_VECTOR_INITIALIZER ;
  tgt_loc = RC_VECTOR_INITIALIZER ;

  rc_vector_zeros(&cur_loc, 2) ;
  rc_vector_zeros(&tgt_loc, 2) ;

  cur_loc.d[X] = 2.0 ;
  cur_loc.d[Y] = 3.0 ;

  rc_vector_print(cur_loc) ;
  rc_vector_print(tgt_loc) ;

  rc_vector_free(&cur_loc) ;
  rc_vector_free(&tgt_loc) ;
} 

