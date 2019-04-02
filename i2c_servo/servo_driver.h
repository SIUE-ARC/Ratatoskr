#define 	SERVO_I2C_BUS 		1

// Base address for the servo driver module

#define 	SERVO_I2C_ADDR 		0x64

// Write buffer size

#define		BUF_SIZE    		12

// PCA9685 registers

#define   	PCA9685_SUBADR1    	0x02
#define         PCA9685_SUBADR2    	0x03
#define         PCA9685_SUBADR3    	0x04
#define  	PCA9685_MODE1		0x00
#define  	PCA9685_MODE2		0x01
#define 	PCA9685_PRESCALE	0xfe

#define		SERVO_0_ON_L		0x06
#define		SERVO_0_ON_H		0x07
#define		SERVO_0_OFF_L		0x08
#define		SERVO_0_OFF_H		0x09

#define		ALL_SERVO_ON_L		0xfa
#define		ALL_SERVO_ON_H		0xfb
#define		ALL_SERVO_OFF_L		0xfc
#define		ALL_SERVO_OFF_H		0xfd

// Useful defines

#define   	TRUE	1
#define		FALSE 	0

// 
// Function declaration

// Set the servo pulse repetition rate

unsigned int setServoFREQ(double) ;

// Set the pulse width of one of the servo channels

unsigned int setServoPW(int, int) ;

// Resets the servo driver

unsigned int resetServoDriver(void) ;
