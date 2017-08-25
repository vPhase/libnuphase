#include "nuphasedaq.h" 

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h> 
#include <stdlib.h>

volatile static int stop = 0; 

static  nuphase_dev_t * dev;
static void catch_interrupt(int signo)
{

  printf("Caught interrupt...\n"); 
  nuphase_cancel_wait(dev); 
  stop =1; 
}


// double_read spidev channel start_addr, end_addr, 
int main(int nargs, char ** args )
{


  uint8_t start_addr; 
  uint8_t end_addr; 
  int ichan; 
  int size; 
  int buffer;
  uint8_t *buf0; 
  uint8_t *buf1; 


  if (nargs < 5) 
  {
    printf("Usage: read_addr spidev channel start_addr end_addr\n"); 
    return 1; 
  }

  ichan = atoi(args[2]); 
  start_addr = atoi(args[3]); 
  end_addr = atoi(args[4]); 
  size = (end_addr-start_addr+1) * NP_NUM_CHUNK * NP_WORD_SIZE; 


  buf0 = malloc(size); 
  buf1 = malloc(size); 

  signal(SIGINT, catch_interrupt); 
  dev =  nuphase_open(args[1],0,60,0,0,0); //no interrupt for now and no threadlocking


  int i = 0; 
  while (!stop) 
  {
    nuphase_sw_trigger(dev); 
    nuphase_buffer_mask_t mask; 
    nuphase_wait(dev,&mask, 1.0,MASTER); 

    if (!mask) 
    {
      fprintf(stderr,"Timed out :( \n"); 
      return 1; 
    }
    buffer = __builtin_ctz(mask); 
    nuphase_read_raw(dev, buffer, ichan, start_addr, end_addr, buf0,MASTER); 
    nuphase_read_raw(dev, buffer, ichan, start_addr, end_addr, buf1,MASTER); 
    nuphase_clear_buffer(dev,1 << buffer); 

    if (memcmp(buf0,buf1,size))
    {
      fprintf(stderr,"Do not match! i=%d, buffer=%d\n",i, buffer); 
      int j; 

      for (j = 0; j < size; j++) 
      {
        if (buf0[j] !=buf1[j])
        {
          printf("\t %04d: %03d  %03d\n", j, buf0[j], buf1[j]); 
        }
      }
    }

    i++; 

  }


  nuphase_close(dev); 


  return 0; 

}
