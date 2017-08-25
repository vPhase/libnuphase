#include "nuphasedaq.h" 
#include <stdio.h> 
#include <inttypes.h>
#include <stdlib.h> 


// read_status spidev
int main(int nargs, char ** args) 
{
  int not_ok;
  nuphase_dev_t * dev;
  nuphase_status_t status; 


  if (nargs < 2) 
  {
    printf("Usage: read_status spidev\n"); 
    return 1; 
  }

  //no interrupt, no locking, default config, 
  dev = nuphase_open(args[1],0,0,0,0,0); 
  not_ok = nuphase_read_status(dev, &status,MASTER); 
  nuphase_close(dev); 

  if (not_ok)
  {
    printf("Something went wrong :(\n"); 
  }
  else
  {
    nuphase_status_print(stdout, &status); 
  }

  return 0; 

}
