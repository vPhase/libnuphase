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
  dev = nuphase_open(args[1],nargs > 2 ? args[2] : 0,60,0,0,0); 
  nuphase_config_t cfg; 
  nuphase_config_t cfg_slave; 
  nuphase_config_init(&cfg,MASTER); 
  nuphase_config_init(&cfg_slave,SLAVE); 
  nuphase_reset(dev,&cfg,nargs > 2 ? &cfg_slave : 0 ,NP_RESET_GLOBAL); 
  nuphase_close(dev); 



  return 0; 

}
