#include "nuphasedaq.h" 

#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>

// read_event spidev hdfile evfile [softwaretrigger=1]

int main(int nargs, char ** args )
{

  nuphase_dev_t * dev;
  nuphase_header_t hd; 
  nuphase_event_t ev; 

  int sw_trigger = 1; 
  if (nargs < 4) 
  {
    printf("Usage: read_event spidev hdfile evfile [software trigger = 1]\n"); 
    return 1; 
  }

  if (nargs > 4) 
  {
    sw_trigger = atoi(args[4]); 
  }

  dev =  nuphase_open(args[1],0,0,0); //no interrupt for now and no threadlocking


  if (sw_trigger) 
  {
    nuphase_sw_trigger(dev); 
  }

  nuphase_buffer_mask_t mask; 
  nuphase_wait(dev,&mask,0.5); 

  struct timespec t0; 
  struct timespec t1; 

  assert(mask); 

  clock_gettime(CLOCK_MONOTONIC,&t0); 
  nuphase_read_single(dev, __builtin_ctz(mask), &hd, &ev); 
  clock_gettime(CLOCK_MONOTONIC,&t1); 

  nuphase_close(dev); 

  printf("Approx time to read out:  %g ms\n", 1000*t1.tv_sec + 1e-6 * t1.tv_nsec - 1000 * t0.tv_sec - 1e-6  * t0.tv_nsec); 

  FILE * f = fopen(args[2],"w"); 
  nuphase_header_write(f,&hd); 
  fclose(f); 

  f = fopen(args[3],"w"); 
  nuphase_event_write(f,&ev); 
  fclose(f); 


  return 0; 

}


