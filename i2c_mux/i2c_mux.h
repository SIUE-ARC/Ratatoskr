//
// We need to enable and disable ports in the MUX
// More than one port may be enabled
//


#define    MUX_I2C_BUS      1
#define    MUX_I2C_ADDR     0x71
#define    ALL_PORTS        0xff

// Routine to enable the specificed port 

void enableMuxPort(uint8_t  portNumber) ;

// Routine to disable the specfied port

void disableMuxPort(uint8_t  portNumber) ;
