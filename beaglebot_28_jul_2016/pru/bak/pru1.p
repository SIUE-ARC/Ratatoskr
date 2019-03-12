// Assembly language code to run on pru 1
// We will 

// Pseudo code
//
// START: 
// Clear the registers
// setup GPIO using OCP port
// Read the delay value set by ARM
// update the status register
// while(!run_flag){                    //Eventually set by pru0
//      update the status register
// }                                    

// MAIN: 
// Send interrupt to pru0
// Read in PWM high periods from PRU 0 into r7 - r10
// Turn all 4 PWM outputs ON using statusReg Byte
// I_LOOP:
// for (i = 4096, i != 0, i--) {  // use r0 for i
//		check encoder edges
//		if (posedge on wheel encoder 1) incr enc1
//		if (posedge on wheel encoder 4) incr enc2
//		if (posedge on wheel encoder 2) incr enc3
//		if (posedge on wheel encoder 3) incr enc4
//		if (pwm1 == 0) clr bit1   else pwm1--
//		if (pwm2 == 0) clr bit2   else pwm2--
//		if (pwm3 == 0) clr bit3   else pwm3--
//		if (pwm4 == 0) clr bit4   else pwm4--
//		for (j = DELAY, j != 0, j--) { } // stall
//              save encoder values 
//              update status reg
//              if (!runFlag) goto PWM_HALT		
// }
// if (button press) goto HALT else goto MAIN
//
// STOP:
// brake motors (Hard or Soft break)
// zero encoder tics
// 
// PWM_WAIT:
// if (button press) goto HALT
// update status reg
// if (run_flag) goto MAIN
// for (j = DELAY; j!=0; i--){}
// goto PWM_WAIT
//
// HALT_PRU1:
// set outputs to low
// set LED to low
// send interrupt to ARM
// halt the unit


#include        "./pru1.h"
.origin		0
.entrypoint	START


// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
START:		        
                        clear_REGS	 // Clear r0-r29 (29 * 4 = 120 bytes)
                        gpio_SETUP       // Setup Linux space GPIO pointers in registers
// Grab the value we need for our delay loop            
// Host would have written it into location 0 of data (pru1Mem)
// delayValue will contain the value
                        read_delay_value
                        led_OFF
// Use sharedMem as a pointer to the PRU shared data RAM ($0001_0000)
// That is how we will transfer results to and from PRU 0
// Setting up the pointer
			ldi		sharedMem, 1
			lsl	        sharedMem, sharedMem, 16
// r0 will be a pointer to pru1 memory ($0000_0000)    
INIT_READ:              update_status_reg
                        //Loop until we get the run flag (1)
                        qbbc    INIT_READ, run_flag
MAIN:		        // Send interrput to PRU 0 to let it know about start of sample period!
                        send_pru0_interrupt
// Need to save the wheel encoder input values from past sample period
// Read in R7 - R10 (PWM high times) from PRU0
// these are writting to shared memory
			read_pwm_values	       // loading r7-r10
                               //Use b0 of the state register to start pwm


// We will check the wheel encoders 4096 times each sample period (50 ms)
// So we need to do in about 3.05 us
                        
			mov	        i, #TIMES	 		// set i = 4096
                        pwm_start
	I_LOOP:         check_encoder_edges
// Wheel encoder #1
        ENC_1_CHK:      enc_cnt         enc1, enc1_bit
// Wheel encoder #2
        ENC_2_CHK:      enc_cnt         enc2, enc2_bit		
// Wheel encoder #3
        ENC_3_CHK:      enc_cnt         enc3, enc3_bit
// Wheel encoder #4
        ENC_4_CHK:      enc_cnt         enc4, enc4_bit
// Now check PWM Timer 
// PWM output #1
        PWM_1_CHK:      pwm_timer       M1_ctrl, pwm1, M1_0, M1_1
// PWM output #2
        PWM_2_CHK:      pwm_timer       M2_ctrl, pwm2, M2_0, M2_1
// PWM output #3
        PWM_3_CHK:      pwm_timer       M3_ctrl, pwm3, M3_0, M3_1
// PWM output #4
        PWM_4_CHK:      pwm_timer       M4_ctrl, pwm4, M4_0, M4_1

// Kill some time before going back around
           
	TIMEKILL:	mov	j, delayValue 	// store the length of delay in r24 (j)
	J_LOOP:		dec     j       		
			qbne	J_LOOP, j, 0
// Save the encoder counter values to shared sharedMemory
			sbbo	&enc1, sharedMem, 16, 16 
// finishing i loop               
// mimic a break for status reg
                        update_status_reg
                        qbbc   STOP, run_flag
// mimic a break for update
                        qbbs   UPDATE, update_flag     
// Decrement i if there is no break
        		dec     i			
			qbne	I_LOOP, i, 0
                        led_TOGGLE
// If user button pressed, then let ARM know we are halting and then halt!
                        qbpr    HALT_PRU1                //Go to halt pru1 if button press
                        qba     MAIN                     //go back to main if no button press
HALT_PRU1:              mov     r30, 0x00000000          //Set PWM's to low         	
                        send_ARM_interrupt               //Send the ARM interrupt	
                        halt	

UPDATE:                 clear_update_flag                 //assumes that you do not want to save enc tics   
STOP:                   pwm_stop                                // Stop PWM
                        led_OFF
//                        zero            &enc1, 16                 // Zero the wheel Encoders
        PWM_WAIT:       qbpr            HALT_PRU1               // If we got a button press in this mode
                        update_status_reg                       // Read in new statusReg Value
                        qbbs            MAIN, run_flag          // If we got a start bit restart
                        mov             j, delayValue           // If not loop here and keep checking
        WAIT_LOOP:      dec             j                       
                        qbne            WAIT_LOOP, j, 0
                        qba             PWM_WAIT

