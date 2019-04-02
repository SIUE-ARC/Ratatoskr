//
// Replicating the Arduino millis()
//

#include  <stdio.h>
#include  <fcntl.h>
#include  <unistd.h>
#include  <sys/ioctl.h>

#include  <linux/i2c-dev.h> //for IOCTL defs
#include  <robotcontrol.h>


unsigned long millis(void) {
  unsigned long  ms ;
  uint64_t  nanos ;

  nanos = rc_nanos_since_epoch() ;
  ms = (unsigned long) (nanos / 1000000) ;

  return ms ;
}

//
// Routine to read bytes using i2c
//

int  wire_read_bytes(uint8_t bus, uint8_t count, uint8_t *buf) {
   int  ret, fd ;

   fd = rc_i2c_get_fd(bus) ;
   ret = read(fd, buf, count) ;
   if (ret != count) return -1 ;
   else return ret ;
}


