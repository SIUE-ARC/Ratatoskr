//
// This is the main program on the BBB
// It launches the motor_control code on PRU 0
// It also launches the wheel encoder / sample period code on PRU 1
//

// Look in standard places

#include <stdio.h>
#include <stdlib.h>

// These can be found in ../include

#include "ROBOTlib.h"
#include "child.h"
#include "motorPRU.h"

//These will eventually be added to the PRU Class
static void *pru0DataMemory;
static unsigned int *pru0DataMemory_int;

static void *pru1DataMemory;
static unsigned int *pru1DataMemory_int;

static void *pruSharedDataMemory;
static unsigned int *pruSharedDataMemory_int;




// ********************************
// Initialization routine
// ********************************

/*void PRUinit(void) {
// Initialize structure used by prussdrv_pruintc_intc   
// PRUSS_INTC_INITDATA is found in pruss_intc_mapping.h 

   tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

// Allocate and initialize memory 

   prussdrv_init ();

// For PRU 0

   prussdrv_open (PRU_EVTOUT_0);

// For PRU 1

   prussdrv_open (PRU_EVTOUT_1);

// Map PRU's INTC 

   prussdrv_pruintc_init(&pruss_intc_initdata);

// Load and execute binary on PRU0 
// Since using C-code for PRU, we need to give START_ADDR 

 	prussdrv_exec_program_at(PRU0, "./text.bin", START_ADDR);

// Load and execute binary on PRU1 

   	prussdrv_exec_program(PRU1, "./pru1.bin");   

// Wait for event completion from PRU 1 
	
	return ;
}
*/
// ****************************
// Routine to clean up 
// ****************************
/*
void PRUcleanup(void) {

///Disable PRU and close memory mappings 

	printf("Disabling the PRUs and exiting\n") ;

   	prussdrv_pru_disable(PRU0);
   	prussdrv_pru_disable(PRU1);

   	prussdrv_exit ();

	return ;
}
*/
// **********************************
// Main program
// **********************************

int main (void) {  

// These are need for the call to "child"
// We will spawn a child process (tclsh) and
// set up two way pipe redirecting stdin and stdout
 	
	FILE 	*read_from, *write_to;
   	int 	childpid ;
	char	str[80] ;

// Number of times we run PRU program

   	int n = 0;

	printf("Running tst-Brd\n") ;

// Perform some initialization
// Calls a bunch of routines from PRU API
// Loads and executes programs in PRU 0 and PRU 1


// Set up GPIO LED and SWITCH for testing
	initPin(SWITCH);
	initPin(LED);
	setPinDirection(SWITCH, IN);
	setPinDirection(LED, OUT);

// Turn our LED on for testing
	
	motorInit();

// Here is a how you can write to PRU data memory
// We are writing to PRU 1 memory here. A count used
// to create a delay

   	prussdrv_map_prumem(PRUSS0_PRU1_DATARAM, &pru1DataMemory);
   	pru1DataMemory_int = (unsigned int *) pru1DataMemory;

   	unsigned int delay_cnt = 1000 ;         //delay factor - 1000 origininally
   	*(pru1DataMemory_int+1) = delay_cnt ;

// First wait for PRU 0 to complete
// Has to received a specified number of interrupts
// from PRU 1

	printf("Waiting for PRU 0 to complete.\n") ;
   	n = prussdrv_pru_wait_event (PRU_EVTOUT_0);  
	printf("PRU 0 program completed, event number %d.\n", n);
	prussdrv_pru_clear_event (PRU_EVTOUT_0,PRU0_ARM_INTERRUPT);
// We will just send a singal to the PRU1 to halt now
// Now wait for PRU 1 to complete
// Person must press the momentary switch on prototype board

	printf("Waiting for PRU 1 to complete.\n") ;
   	n = prussdrv_pru_wait_event (PRU_EVTOUT_1);  
	printf("PRU 1 program completed, event number %d.\n", n);
	prussdrv_pru_clear_event (PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);

// Here is how we can read from shared data memory
// We'll read the PWM values and the encoder counter values
// from the PRU shared data memory

  	prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &pruSharedDataMemory);
   	pruSharedDataMemory_int = (unsigned int *) pruSharedDataMemory;
	
	unsigned int pwm_1, pwm_2, pwm_3, pwm_4 ;
	unsigned int enc_1, enc_2, enc_3, enc_4 ;

	pwm_1 = *(pruSharedDataMemory_int + 0) ;
	pwm_2 = *(pruSharedDataMemory_int + 1) ;
	pwm_3 = *(pruSharedDataMemory_int + 2) ;
	pwm_4 = *(pruSharedDataMemory_int + 3) ;

	enc_1 = *(pruSharedDataMemory_int + 4) ;
	enc_2 = *(pruSharedDataMemory_int + 5) ;
	enc_3 = *(pruSharedDataMemory_int + 6) ;
	enc_4 = *(pruSharedDataMemory_int + 7) ;


	printf("PWM 1 is : %u\n", pwm_1) ;
	printf("PWM 2 is : %u\n", pwm_2) ;
	printf("PWM 3 is : %u\n", pwm_3) ;
	printf("PWM 4 is : %u\n", pwm_4) ;

	printf("ENC 1 is : %u\n", enc_1) ;
	printf("ENC 2 is : %u\n", enc_2) ;
	printf("ENC 3 is : %u\n", enc_3) ;
	printf("ENC 4 is : %u\n", enc_4) ;


// Clean up our mess like mom taught us!

	motorCleanup();

// Here is how we can spawn a child process
/*
   	childpid = start_child("tclsh", &read_from, &write_to);

// Tell tclsh to source the tcl script
// Anything sent to us from tcl script should be printed to screen

//  	fprintf(write_to, "set a 2 ; set b 3; puts [expr $a + $b] \n") ;
// 	if (fgets(str, 80, read_from) > 0)  printf("answer is %s\n", str) ;

// Test the sonar module

    test_sonar() ;

// Go out for a test drive

//    test_drive() ;

// Test the servo

//	test_servo() ;

//  Turn LED off on PSOC board
//  Then turn on on for 5 seconds

   	turnLED(OFF) ;
	turnLED(ON) ;
 	pauseSec(5) ;
	turnLED(OFF) ;

*/
    return 0;
}



