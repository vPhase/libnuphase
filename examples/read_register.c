#include "nuphasedaq.h" 

#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>

// read_register spidev  reg

int main(int nargs, char ** args )
{

  int reg = 7; 
  if (nargs < 2) 
  {
    printf("Usage: read_register spidev [reg=7]\n"); 
    return 1; 
  }

  if (nargs > 2) 
    reg = atoi(args[2]); 


  nuphase_dev_t * dev =  nuphase_open(args[1],0,60,0); //no interrupt for now and no threadlocking

  printf("Sending trigger\n"); 
  nuphase_sw_trigger(dev); 
  uint8_t result[4]; 
  nuphase_read_register(dev,reg,result,MASTER); 
  printf("READ REGISTER %d, GOT: 0x%x 0x%x 0x%x 0x%x\n",reg, result[0], result[1], result[2], result[3]); 
  nuphase_close(dev); 

  return 0; 
}
