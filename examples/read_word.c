#include "nuphasedaq.h" 
#include <stdio.h> 
#include <stdlib.h> 


// read_word spidev
int main(int nargs, char ** args) 
{
  uint8_t bytes[4]; 
  int i,status;
  nuphase_dev_t * dev;


  if (nargs < 2) 
  {
    printf("Usage: read_word spidev \n"); 
    return 1; 
  }

  //no interrupt, no locking, default config, 
  dev = nuphase_open(args[1],0,60,0,0,0); 
  status = nuphase_read(dev, bytes,MASTER); 
  nuphase_close(dev); 

  if (status)
  {
    printf("Something went wrong\n"); 
    return 1; 
  }

  for (i = 0; i < 4; i++) 
  {
    printf("Got:(%x,%x,%x,%x)\n", bytes[0], bytes[1], bytes[2], bytes[3]); 
  }


  return 0; 
}
