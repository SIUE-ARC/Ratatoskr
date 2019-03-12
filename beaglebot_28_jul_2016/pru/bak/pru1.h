#define         TIMES                           4096                    // 12-bit PWM

#define         PRU_R31_VEC_VALID               32                      // allows notification of program completion
#define         PRU_EVTOUT_1                    4                       // event number that is sent back for PRU 1 to ARM interrupt
#define         PRU0_PRU1_INTERRUPT             17                      // PRU0->PRU1 interrupt number
#define         PRU1_PRU0_INTERRUPT             18                      // PRU1->PRU0 interrupt number
#define         ARM_PRU1_INTERRUPT              37			//  ARM->PRU1 interrupt number

#define		M1_ctrl				r29.b0.t2
#define		M2_ctrl				r29.b0.t4
#define		M3_ctrl				r29.b0.t0
#define		M4_ctrl				r29.b0.t6

#define		M1_0				r30.t2
#define		M1_1				r30.t3
#define		M2_0				r30.t4
#define		M2_1				r30.t5
#define		M3_0				r30.t0
#define		M3_1				r30.t1
#define		M4_0				r30.t6
#define		M4_1				r30.t7

#define		enc1_bit			r11.b1.t0
#define		enc2_bit			r11.b1.t1
#define		enc3_bit			r11.b1.t2
#define		enc4_bit			r11.b1.t3

#define         run_flag                        r29.b1.t0
#define         update_flag                     r29.b1.t2

// Define register aliaies
#define		enc1				r1
#define		enc2				r2
#define		enc3				r3	
#define		enc4				r4
#define		encNEW				r9
#define		encOLD				r10
#define		encEDGE				r11
#define		pwm1				r5
#define		pwm2				r6
#define		pwm3				r7
#define		pwm4				r8
#define		j				r24
#define		i				r25
#define		holder				r26	//Can be used to temporally hold values if needed
#define		delayValue			r27
#define		sharedMem			r28
#define         pru1Mem                         r23
#define 	stateReg			r29
#define		nopReg				r29.b3


//// Define linux space GPIO access
#define         GPIO0                           0x44e07000
#define         GPIO1                           0x4804C000
#define         GPIO_CLEARDATAOUT               0x190                   //Clearing GPIO
#define         GPIO_SETDATAOUT                 0x194                   //Setting GPIO
#define         GPIO_DATAOUT                    0x138                   //reading GPIO
#define         GPIO1_15_MASK                   0x80                    //SWITCH
#define         GPIO1_12_MASK                   0x10                    //LED
#define         GPIO_LED_STATE                  r14
#define         GPIO_BUTTON                     r15
#define         GPIO_LED                        r16
#define         read_gpio1                      r17
#define         set_gpio1                       r18
#define         clr_gpio1                       r19

// Define Macros
		
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Macro used to enable to OCP port
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^		
.macro		ocp_port_ENABLE
             		lbco    r0, C4, 4, 4     // load SYSCFG reg into r0 (use c4 const addr)
	         	clr     r0, 4            // clear bit 4 (STANDBY_INIT)
	         	sbco    r0, C4, 4, 4     // store the modified r0 back at the load addr
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Macro to set up the GPIO utils
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		gpio_SETUP
			ocp_port_ENABLE
                        mov     read_gpio1, GPIO1 | GPIO_DATAOUT
                        mov     set_gpio1, GPIO1 | GPIO_SETDATAOUT
                        mov     clr_gpio1, GPIO1 | GPIO_CLEARDATAOUT
                        set     GPIO_LED, 12
.endm

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// No operation
// (1 clock cycle)
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		NO_OP
			mov    nopReg, nopReg
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Clear All registers
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro      	clear_REGS
	        zero    &r0, 120	
.endm	

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Turn the GPIO LED on (p8.12, GPIO1[12], GPIO:44)
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		led_ON
//	                set    GPIO_LED.b1, GPIO_LED.b1, GPIO1_12_MASK
                        set    GPIO_LED_STATE, 12
                        sbbo   GPIO_LED, set_gpio1, 0, 4
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Turn the GPIO LED off (p8.12, GPIO1[12], GPIO:44)
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		led_OFF
//                        set     GPIO_LED.b1, GPIO_LED.b1, GPIO1_12_MASK
                        clr     GPIO_LED_STATE, 12
                        sbbo    GPIO_LED, clr_gpio1, 0, 4
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Toggle the GPIO LED (p8.12, GPIO1[12], GPIO:44)
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		led_TOGGLE
                        xor      GPIO_LED_STATE.b1, GPIO_LED_STATE.b1, 1<<4
                        qbbc     L0, GPIO_LED_STATE, 12            
                        led_ON
                        qba      L1
L0:			led_OFF                      
L1:                           
.endm 

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// QBPR : Quick branch if button press
//
// Usage: 
//	qbpr LOCATION
//
// Branches to target location if GPIO button is pressed
//	(Green Button, p8.15, GPIO1[15], GPIO:47)
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		qbpr	
.mparam   	LOCATION 
	                lbbo   GPIO_BUTTON, read_gpio1, 0, 4
                        qbbs   LOCATION, GPIO_BUTTON.t15
.endm

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// movi32 : Move a 32bit value to a register
//
// Usage:
//     movi32   dst, src    
//
// Sets dst = src. Src must be a 32 bit immediate value.
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro  	movi32               
.mparam 	dst, src
           		mov     dst.w0, src & 0xFFFF
           		mov     dst.w2, src >> 16
.endm

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// dec: Decrement value/register
// 
// Usage:
//	dec value
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro       	dec      
.mparam      	value           
             		sub     value, value, #1
.endm  

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// inc: Increment value/register
// 
// Usage:
//	inc value
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro	     	inc
.mparam		value
			add	value,value, #1
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// read_pwm_values
//
// Usage:
//	read in PWM values from sharedMemory
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		read_pwm_values
			lbbo	pwm1, sharedMem, 0, 16
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// read_delay_value
//
// Description:
//	read in the delay loop value from sharedMemory (written by host)
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		read_delay_value
                        mov 	pru1Mem, 0x00000000
			lbbo	delayValue, pru1Mem, 4, 4	
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// update_status_reg
//
// Description:
//	update the status register (r29) value in pru1 Memory
//	The state value can be updated by pru0 and the ARM
//	system if necesary it is used to determine wheel direction
//	braking system, if a update of PWM values need to happen,
//	or if the system should stop
//
//	Status byte structure:	Wheel control:     b0
//				Status Flags: 	   b1-b2
//					b1.t0 - run flag   (1 if in run mode) 
//					b1.t1 - brake flag  (1 if in hard brake)
//					b1.t2 - update flag (1 if an update for pwm)
//				Nop (stays zero):  b3
//
//	TODO: Make sure that we can reliably update the value 
//		without a race condition
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro 		update_status_reg
			lbbo	stateReg, pru1Mem, 0, 4
//Work in progress
.endm
		

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// pwm_timer
//
// Description:
//	This decrements the PWM register given the motor 
//	timer register and will stop the pwm signal if 
//	necessary
//
// Usage:
// 	pwm_timer	M1_ctrl, pwm1, M1_1, M1_1, NEXT_LINE 
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		pwm_timer
.mparam		M0_ctrl, pwm0, M0_0, M0_1
			
				qbbc 	CCW, M0_ctrl 		//Check if we are clockwise or counter clockwise
		CW:		qbne 	PWM_JMP, pwm0, 0	//Check if time to bring low
				clr	M0_0			//Set bit low
				qba	NEXT			//Jump to the next instruction

		CCW:		qbne	PWM_JMP, pwm0, 0	//counter clockwise case
				clr	M0_1			
				qba	NEXT

		PWM_JMP: 	dec pwm0			//If it is not time to bring low decrement
				NO_OP				//NO_OP to mimic jump
                NEXT:
.endm

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// enc_cnt
//
// Description:
//      Reads the encoder tics and increments if need be
//
// Usage:
//      enc_cnt		enc1, enc1_bit, NEXT_LINE
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.macro		enc_cnt
.mparam		enc0, enc0_bit
				qbbc	ENC_JMP, enc0_bit	//If clear jump to ENC_JMP
				inc	enc0			//If there is an edge increment
				qba	NEXT			//Jump to next
		ENC_JMP:	NO_OP				//Mimic inc
				NO_OP				//Mimic jump
                NEXT:
.endm



// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// check_encoder_edges
//
// Description:
//      Transfers the old encoder values read the new ones
//	and xor to see if there is an edge
//
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.macro		check_encoder_edges
			mov	encOLD, encNEW
			mov	encNEW.b1, r31.b1
			xor	encEDGE, encNEW, encOLD
		
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// pwm_start
//
// Description:
//	Uses the state register to start the pwm values
//	stored in b0
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		pwm_start
			mov	r30,	r29
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// store_encoder_values
//
// Description:
//	Stores the current encoder values to shared sharedMemory
//
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		store_encoder_values
			sbbo	&enc1, sharedMem, 16, 16
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// pwm_stop
// Description:
//	Stops the pwm by setting outputs to zero or to one
//	depending on the brake bit
//
// Usage:	
//	stop_pwm  NEXT_LINE
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		pwm_stop	

				qbbs	HARD_BR, r29.b1.t1
				mov	r30.b0, 0x00
				qba	NEXT
		HARD_BR:	mov	r30.b0, 0xFF
				NO_OP			//mimic qba
                NEXT:
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// send_ARM_interrupt
//
// Description:
//	Send an interrupt to the arm from pru1
//
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		send_ARM_interrupt
			mov	r31.b0, PRU_R31_VEC_VALID | PRU_EVTOUT_1
.endm

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// send_pru0_interrupt
//
// Description:
//	Send an interrupt to pru0 from pru1
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		send_pru0_interrupt
			ldi	r31, PRU1_PRU0_INTERRUPT + 16

.endm

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// init
//
// Description:
//      setup values and delays
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro          initialize
                clear_REGS
                gpio_SETUP
                read_delay_value
                ldi     sharedMem, 1
                lsl     sharedMem, sharedMem, 16
.endm

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// clear_update_flag
//
// Description:
//	removes the update flag from memory so there isn't
//	 a stop loop
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.macro		clear_update_flag
		clr	update_flag
		sbbo	stateReg, pru1Mem, 0, 4
.endm
