#include "nuphasedaq.h" 
#include <stdio.h> 
#include <stdlib.h> 


// write_word spidev byte0 byte1 byte2 byte3 
int main(int nargs, char ** args) 
{
  uint8_t bytes[4]; 
  int i,status;
  nuphase_dev_t * dev;


  if (nargs < 6) 
  {
    printf("Usage: write_word spidev b0 b1 b2 b3\n"); 
    return 1; 
  }

  //no interrupt, no locking, default config, 
  dev = nuphase_open(args[1],0,0,0); 

  for (i = 0; i < 4; i++)
  {
    bytes[i] = strtol(args[i+2], 0, 16); 
  }

  for (i = 0; i < 4; i++) 
  {
    printf("Sending:(%x,%x,%x,%x)\n", bytes[0], bytes[1], bytes[2], bytes[3]); 
  }

  status = nuphase_write(dev, bytes); 
  printf(status ? "Success!\n" : "Something went wrong\n"); 
  nuphase_close(dev); 

  return 0; 
}
