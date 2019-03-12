//
// This is the motor control code that will run on PRU 0
// 
#include <stdint.h>
#include "pru_cfg.h"
#include "pru_intc.h"
#include "fp.h"

volatile register uint32_t __R30;
volatile register uint32_t __R31;

/* Mapping Constant table register to variable */

volatile pruCfg CT_CFG __attribute__((cregister("PRU_CFG", near), peripheral));
volatile far pruIntc CT_INTC __attribute__((cregister("PRU_INTC", far), peripheral));


//  Defines 

//#define PRU addresses
#define PRU0
#define HOST1_MASK		(0x80000000)
#define HOST0_MASK		(0x40000000)
#define PRU0_PRU1_EVT		(16)
#define PRU1_PRU0_EVT		(18)
#define PRU0_ARM_EVT		(34)
#define	PRU0_MEM		(0x00000000)
#define PRU1_MEM		(0x00002000)
#define SHARE_MEM		(0x00010000)

#define M_RUN			(0x00000100)
#define M_HARD_BREAK		(0x00000200)
#define M_UPDATE		(0x00000400)
#define M1_CW			(0x00000004)
#define M1_CCW			(0x00000008)
#define M2_CW			(0x00000010)
#define M2_CCW			(0x00000020)
#define M3_CW			(0x00000001)
#define M3_CCW			(0x00000002)
#define M4_CW			(0x00000040)
#define M4_CCW			(0x00000080)

// Bit 15 is P8-11
// Bit 14 is p8-16
// Bit 07 is p9-25
// Bit 05 is p9-27

#define TOGGLE_PRU_LED			(__R30 ^= (0x00008000)) //Bit 15
#define OFF_PRU_LED			(__R30 &= (0xFFFF7FFF))
#define ON_PRU_LED			(__R30 |= (0x00008000))
#define DISABLE_DRV			(__R30 |= (0x00000020)) //Bit 5 Neg logic
#define ENABLE_DRV			(__R30 &= (0xFFFFFFDF)) //Bit 5 Neg logic
#define PRU_SW_VALUE			(__R31  & (0x00004000))	//Bit 14
#define	ACC_IN1_VAL			(__R31  & (0x00000080))	//Bit 7

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Subroutine to perform initialization

void init(void) {

/* Configure GPI and GPO as Mode 0 (Direct Connect) */
//	TODO: if the userspace GPIOs are to be used this will
//		need to change currently is freezes the pru
//	CT_CFG.GPCFG0 = 0x0000;

/* Clear GPO pins */
	//__R30 &= 0xFFFF0000;

/* Clear interrupt event 18 */
	
	CT_INTC.SICR = PRU1_PRU0_EVT ; 	


	return ;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// The main routine

void main() {

// Used by real-time scheduler
// NOT USING THESE AT THE MOMENT
	//unsigned int temp, TimeSlot, SliceCounter ;
	//TimeSlot = 0 ;
	//SliceCounter = 0 ;
	
// Perform initialization

//	init() ;
// TODO: change this so that any init values for memory pointing, Globals may be a good idea		
//
// Writing the 4 PWM values into the shared memory block (address is 0x0001_0000)
// PRU 1 will grab them and place the values intr R7 - R10

	int	* sharedMem ;
	int	* pru1Mem;
	pru1Mem	  = (int *) PRU1_MEM ;	
	sharedMem = (int *) SHARE_MEM ;

// Store the 4 PWM values into shared memory

	*sharedMem = 2048;
	*(sharedMem+1) = 2048  ;
	*(sharedMem+2) = 800  ;
	*(sharedMem+3) = 1600 ;
// Store the PRU1 status value
	*pru1Mem = (M_RUN | M1_CW | M2_CCW | M3_CW | M4_CW);

// We will use i to count intertupts
// After seeing specified number we quit
// We will also read wheel encoder values

	volatile int enc1, enc2, enc3, enc4 ;

	int i = 0;
	int j = 1;
	int flag = 1;
	int bState = 0;
	//PRU_LED_ON;	
// Enable the Motor Driver signals
	ENABLE_DRV;
// Clear the first interrupt
	while(flag == 1){
		if(__R31 & HOST0_MASK){
			CT_INTC.SICR = PRU1_PRU0_EVT;
			flag = 0;
		}
	}
	//__R31 = 35;
	flag = 1;
		//Send an interrupt to ARM
// Start the loop
	
	while (flag == 1) { 
		if(i >= 7){
                        flag = 0;
                }		
// Wait for the start of the new sample period i.e. interrupt from PRU 1
		if (__R31 & HOST0_MASK) {

// Clear interrupt event 18 
			CT_INTC.SICR = PRU1_PRU0_EVT ;

// Read the wheel encoder counters 
			enc1 = *(sharedMem+4) ;
			enc2 = *(sharedMem+5) ;
			enc3 = *(sharedMem+6) ;
			enc4 = *(sharedMem+7) ;
		} 
		if (PRU_SW_VALUE){
			if(bState == 0){
				bState = 1;
				i++;
				DISABLE_DRV;
				if(i == 1 || i == 5) {
					*pru1Mem = (M_RUN | M_HARD_BREAK | M1_CCW | M2_CW | M3_CW | M4_CW); 
					if(i == 1){
						*sharedMem = 200;
						*(sharedMem+1) = 200;
					}
					if(i == 5){
						*sharedMem = 3072;
						*(sharedMem+1) = 3072;
					}
				}
				else{
					if(i==3){ *pru1Mem = *pru1Mem & ~M_RUN; } //Try cleaing the start bit
					else { 
						*pru1Mem = (M_RUN | M_HARD_BREAK | M_UPDATE | M1_CW | M2_CCW | M3_CW | M4_CW); //Currently update clears encoder tics
				 		*sharedMem = 2048;
				        	*(sharedMem+1) = 2048  ;
					}
				}
				ENABLE_DRV;	
				while(PRU_SW_VALUE){}                                                           //Trying to make sure that the button has been released
			}
		}	 
		if(i >= 4){
			ON_PRU_LED;
		}
		//Trying to prevent bounces
		if(bState == 1){
			j++;
		}
		if(j == 50000){
			bState = 0;
			j = 0;
		
		}
// Where we would call the PID routines

	}
//	*(sharedMem+4) = 0;
//	*(sharedMem+5) = 0;
// 	*(sharedMem+6) = 0;
//	*(sharedMem+7) = 0;
   	DISABLE_DRV;
   	OFF_PRU_LED;
	__R31 = 35;
   	__halt();                        // halt the PRU

}



