//
// Functional prototypes

#define     PI            3.14159

#define     LOOPBACK    1
#define     BROADCAST   2
#define     VAD         3
#define     SINE        4

// Declare a structure to hold the GUI variables

typedef struct {
    int      exitFlag ;
    float    sine_freq ;
    float    lpf_corner ;
    int      opMode ;
    int      ch0Ena ;
    int      ch1Ena ;
    int      ch2Ena ;
    int      ch3Ena ;
    float    vol0 ;
    float    vol1 ;
    float    vol2 ;
    float    vol3 ;
    int      VADmode ;
    float    VAD_frame ;
    float    VAD_thresh ;
    float    pwm_p ;
    float    bits ;
    int      headset ;
    float    fs ;
} GUIvars_t ;

void initOsc(osc_t * osc) ;
void initIIR(iir_1_t * filt) ;

// void designLPF(int ntaps, float cornerFreq) ;

void GPIOinit(void) ;
void dataDump(void) ;
void getGUIvars(char *str) ;
void configPRU(void) ;
void memoryDump(void) ;
