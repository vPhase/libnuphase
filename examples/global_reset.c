#include "nuphasedaq.h" 
#include <stdio.h> 
#include <inttypes.h>
#include <stdlib.h> 


// global_reset spidev
int main(int nargs, char ** args) 
{
  nuphase_dev_t * dev;


  if (nargs < 2) 
  {
    printf("Usage: global_reset spidev [spidev[2])\n"); 
    return 1; 
  }


  //no interrupt, no locking, default config, 
  dev = nuphase_open(args[1],nargs > 2 ? args[2] : 0,0,0); 
  nuphase_reset(dev,NP_RESET_GLOBAL); 
  nuphase_close(dev); 



  return 0; 

}
