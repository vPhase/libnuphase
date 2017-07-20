#include "nuphasedaq.h" 

#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>

// read_many spidev [swtrig=1] [hdfile] [evfile] 

volatile static int stop = 0; 

static void catch_interrupt(int signo)
{

  printf("Caught interrupt...\n"); 
  stop =1; 
}


int main(int nargs, char ** args )
{

  nuphase_dev_t * dev;
  nuphase_header_t hd[4]; 
  nuphase_event_t ev[4]; 


  int sw_trigger = 1; 
  if (nargs < 2) 
  {
    printf("Usage: read_event spidev [software trigger = 1] [hdfile=headers.dat]  evfile = [events.dat]\n"); 
    return 1; 
  }

  if (nargs > 2) 
  {
    sw_trigger = atoi(args[2]); 
  }

  FILE * fhd = fopen(nargs > 3 ? args[3]: "headers.dat" ,"w"); 
  FILE * fev = fopen(nargs > 4 ? args[4]: "events.dat" ,"w"); 


  signal(SIGINT, catch_interrupt); 
  dev =  nuphase_open(args[1],0,0,0); //no interrupt for now and no threadlocking

  nuphase_set_event_number_offset(dev,0); 

  printf("Starting event loop... ctrl-c to cancel!\n"); 

  int nev = 0; 
  struct timespec start; 
  struct timespec now; 

  clock_gettime(CLOCK_REALTIME,&start); 

  while (!stop)
  {
    if (sw_trigger) 
    {
        nuphase_sw_trigger(dev); 
        nuphase_sw_trigger(dev); 
        nuphase_sw_trigger(dev); 
        nuphase_sw_trigger(dev); 
    }

    int nevents = nuphase_wait_for_and_read_multiple_events(dev,  &hd, &ev); 
    clock_gettime(CLOCK_REALTIME,&now); 
    nev+= nevents; 
    double hz = nev / (now.tv_sec - start.tv_sec + now.tv_nsec *1.e-9 - start.tv_nsec*1.e-9); 
    printf("Just read %d events (total= %d, avg rate = %f Hz)\n",nevents, nev, hz); 
    int i; 
    for (i = 0; i < nevents; i++)
    {
      nuphase_event_write(fev,&ev[i]); 
      nuphase_header_write(fhd,&hd[i]); 

    }

  }
  nuphase_close(dev); 

  fclose(fhd); 
  fclose(fev); 

  return 0; 

}


