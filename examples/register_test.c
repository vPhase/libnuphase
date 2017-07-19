#include "nuphasedaq.h" 

#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>

// register_test spidev 

int main(int nargs, char ** args )
{

  int n = 10; 
  if (nargs < 2) 
  {
    printf("Usage: register_test spidev [n=10]\n"); 
    return 1; 
  }

  if (nargs > 2) 
    n = atoi(args[2]); 


  nuphase_dev_t * dev =  nuphase_open(args[1],0,0,0); //no interrupt for now and no threadlocking

  int i ;
  for (i = 0; i < n; i++) 
  {
    printf("Sending trigger #%d:\n",i); 
    nuphase_sw_trigger(dev); 
    int j; 
    for (j = 0; j <= 18; j++) 
    {
        uint8_t result[4]; 
        nuphase_read_register(dev,j,result); 
        printf("READ REGISTER %d, GOT: 0x%x 0x%x 0x%x 0x%x\n",j, result[0], result[1], result[2], result[3]); 
    }
    sleep(1); 
  }

  nuphase_close(dev); 

  return 0; 
}
