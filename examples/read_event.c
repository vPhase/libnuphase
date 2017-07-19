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
    printf("Usage: read_event spidev [software trigger = 1] [hdfile=head.dat]  evfile = [ev.dat]\n"); 
    return 1; 
  }

  if (nargs > 2) 
  {
    sw_trigger = atoi(args[2]); 
  }

  FILE * fhd = fopen(nargs > 3 ? args[3]: "head.dat" ,"w"); 
  FILE * fev = fopen(nargs > 4 ? args[4]: "ev.dat" ,"w"); 


  dev =  nuphase_open(args[1],0,0,0); //no interrupt for now and no threadlocking

  nuphase_set_event_number_offset(dev,0); 

  if (sw_trigger) 
  {
      nuphase_sw_trigger(dev); 
      usleep(100e3); 
  }

  struct timespec t0; 
  struct timespec t1; 


  clock_gettime(CLOCK_MONOTONIC,&t0); 
  nuphase_read_single(dev, 0,  &hd, &ev); 
  clock_gettime(CLOCK_MONOTONIC,&t1); 

  printf("Approx time to read out:  %g ms\n", 1000*t1.tv_sec + 1e-6 * t1.tv_nsec - 1000 * t0.tv_sec - 1e-6  * t0.tv_nsec); 

  nuphase_header_write(fhd,&hd); 
  nuphase_event_write(fev,&ev); 

  nuphase_header_print(stdout, &hd); 
  nuphase_event_print(stdout, &ev, ','); 

  nuphase_close(dev); 
  fclose(fhd); 
  fclose(fev); 

  return 0; 

}


