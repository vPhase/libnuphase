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
  if (nargs < 2) 
  {
    printf("Usage: read_event spidev [software trigger = 1]\n"); 
    return 1; 
  }

  if (nargs > 2) 
  {
    sw_trigger = atoi(args[2]); 
  }

  dev =  nuphase_open(args[1],0,0,0); //no interrupt for now and no threadlocking

  nuphase_set_event_number_offset(dev,0); 
  int i; 

  for (i = 0; i < 4; i++) 
  {
    if (sw_trigger) 
    {
      nuphase_sw_trigger(dev); 
      usleep(100e3); 
      nuphase_sw_trigger(dev); 
      usleep(100e3); 
      nuphase_sw_trigger(dev); 
      usleep(100e3); 
      nuphase_sw_trigger(dev); 
    }

   // nuphase_buffer_mask_t mask; 
  //  nuphase_wait(dev,&mask,0.5); 

    struct timespec t0; 
    struct timespec t1; 

  //  assert(mask); 

    uint8_t result[4]; 
    nuphase_read_register(dev,7,result); 
    printf("READ REGISTER 7, GOT: %x %x %x %x\n", result[0], result[1], result[2], result[3]); 
    nuphase_read_register(dev,10,result); 
    printf("READ REGISTER 10, GOT: %x %x %x %x\n", result[0], result[1], result[2], result[3]); 
    nuphase_read_register(dev,12,result); 
    printf("READ REGISTER 12, GOT: %x %x %x %x\n", result[0], result[1], result[2], result[3]); 

    clock_gettime(CLOCK_MONOTONIC,&t0); 
    nuphase_read_single(dev, i,  &hd, &ev); 
    clock_gettime(CLOCK_MONOTONIC,&t1); 

    printf("Approx time to read out:  %g ms\n", 1000*t1.tv_sec + 1e-6 * t1.tv_nsec - 1000 * t0.tv_sec - 1e-6  * t0.tv_nsec); 
    printf("hd->eventNumber: %llu\n", hd.event_number); 
    printf("hd->trig_time: %llu\n", hd.trig_time); 
    printf("hd->trig_type: %d\n", hd.trig_type); 
  }
  nuphase_close(dev); 
  /*
  FILE * f = fopen(args[2],"w"); 
  nuphase_header_write(f,&hd); 
  fclose(f); 

  f = fopen(args[3],"w"); 
  nuphase_event_write(f,&ev); 
  fclose(f); 
  */

  return 0; 

}


