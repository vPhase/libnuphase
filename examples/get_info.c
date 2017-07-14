#include "nuphasedaq.h" 
#include <stdio.h> 
#include <inttypes.h>
#include <stdlib.h> 


// get_info spidev
int main(int nargs, char ** args) 
{
  int status;
  nuphase_dev_t * dev;
  nuphase_fwinfo_t fw; 


  if (nargs < 2) 
  {
    printf("Usage: get_info spidev\n"); 
    return 1; 
  }

  //no interrupt, no locking, default config, 
  dev = nuphase_open(args[1],0,0,0); 
  status = nuphase_fwinfo(dev, &fw); 
  nuphase_close(dev); 

  if (status)
  {
    printf("Something went wrong :(\n"); 
  }
  else
  {
    printf("FW VER: %u\n",fw.ver); 
    printf("FW DATE: %u\n",fw.date); 
    printf("BOARD DNA: 0x %" PRIu64 "\n",fw.dna); 
  }


  return 0; 

}
