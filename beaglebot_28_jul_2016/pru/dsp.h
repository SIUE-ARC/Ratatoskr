//
//  Defines 
//

// Operating Modes

#define  LOOPBACK     1
#define  BROADCAST    2
#define  VAD          3
#define  SINE         4

// Fixed point operationss

#define  FADD(op1,op2)      ( (op1) + (op2) )
#define  FSUB(op1,op2)      ( (op1) - (op2) )
#define  FMUL(op1,op2,q)    ((int32_t) (((int64_t) ((int64_t) (op1) * (int64_t) (op2))) >> q))

// #define  FDIV(op1,op2,q)    ( (int32_t) (((int64_t)(op1) << q)/ ((int64_t) op2 )) )

// Convert from a q1 format to q2 format

#define  FCONV(op1,q1,q2)     (((q2) > (q1)) ? ((op1) << ((q2)-(q1))) : ((op1) >> ((q1)-(q2))))

// Convert a float to a fixed-point representation in q format

#define  TOFIX(op1, q)       ((int32_t) ((op1) * ((float) (1 << (q)))))

// Convert a fixed-point number back to a float

#define  TOFLT(op1, q)       ( ((float) (op1)) / ((float) (1 << (q))) )

// Misc defines

#define PRU0
#define HOST1_MASK		(0x80000000)
#define HOST0_MASK		(0x40000000)

//#define PRU0_PRU1_EVT	(16)

#define PRU1_PRU0_EVT		(18)

// Bit 3 is P9-28 

#define TOGGLE_LED   (__R30 ^= (1 << 3))

// The magic pi

#define   PI     3.14159

// Fixed-point Q values

#define   Q14        14
#define   Q15        15
#define   Q28        28
#define   Q30        30
#define   Q20        20

// Shared memory address

#define   PRU_SHARED_MEM_ADDR     0x00010000  

// GPIO bank addresses

#define      GPIO0                0x44e07000
#define      GPIO1                0x4804c000
#define      GPIO2                0x481ac000
#define      GPIO3                0x481ae000          

// These can be OR'ed with the above bank addresses

#define      GPIO_DATAOUT         0x13C   
#define      GPIO_DATAIN          0x138   
#define      GPIO_CLEARDATAOUT    0x190    
#define      GPIO_SETDATAOUT      0x194  

#define      TP_PIN       27
#define      LED_PIN      26  

#define      OFF          0
#define      ON           1
//
// Ears Library function prototype declarations
//

void       initGPIO(void) ;
void       GPIO0pin(int pin, int value) ;
void       blinkLED(void) ;
 
int16_t    *pwrap(uint32_t bufLen, int16_t * buf, int16_t * ptr) ;
// int16_t    fir(int M,  int16_t *buf,  int16_t *ptr, int Ntaps, int16_t *h) ;
void       initPRU(void) ;
int32_t    sineGen(osc_t *osc) ;       
void       storeInput(void) ;
int16_t    iir_1(iir_1_t * filt) ;
void       updatePointers(void) ;
void       doLPF(void) ;
int32_t    measureEnergy(uint32_t M, int16_t *data, int32_t past_energy, int16_t *ptr, uint32_t N) ;
void       doEnergy(void) ;

// Routine for the 4 operating modes

void doMode(int mode) ;

