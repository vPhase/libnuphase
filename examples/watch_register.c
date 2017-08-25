#include "nuphasedaq.h" 

#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>

// watch_register spidev  reg
//
volatile static int stop = 0; 

nuphase_dev_t * dev;
static void catch_interrupt(int signo)
{

  printf("Caught interrupt...\n"); 
  nuphase_cancel_wait(dev); 
  stop =1; 
}


int main(int nargs, char ** args )
{

  int reg = 7; 
  if (nargs < 2) 
  {
    printf("Usage: watch_register spidev [reg=7]\n"); 
    return 1; 
  }

  if (nargs > 2) 
    reg = atoi(args[2]); 


  nuphase_dev_t * dev =  nuphase_open(args[1],0,0,0,0,0); //no interrupt for now and no threadlocking

  signal(SIGINT, catch_interrupt); 
  int ibuf = 0; 

  while(!stop) 
  {
    printf("Sending 3 triggers\n"); 
    nuphase_sw_trigger(dev); 
    nuphase_sw_trigger(dev); 
    nuphase_sw_trigger(dev); 
    uint8_t result[4]; 
    nuphase_read_register(dev,reg,result,MASTER); 
    printf("READ REGISTER %d, GOT: 0x%x 0x%x 0x%x 0x%x\n",reg, result[0], result[1], result[2], result[3]); 
    nuphase_clear_buffer(dev, 0xf); 
    ibuf= (ibuf + 1 ) %4; 
  }
  nuphase_close(dev); 

  return 0; 
}
