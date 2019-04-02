//
// Arduino
//

#define   I2C_BUS    1

unsigned long millis(void) ;

int  wire_read_bytes(uint8_t bus, uint8_t count, uint8_t *buf) ;
