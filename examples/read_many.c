#include "nuphasedaq.h" 

#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>
#include <string.h>

// read_many spidev [swtrig=1] [hdfile] [evfile] 

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

  nuphase_header_t hd[4]; 
  nuphase_event_t ev[4]; 


  int sw_trigger = 1; 
  int calpulse = 0;
  if (nargs < 2) 
  {
    printf("Usage: read_event spidev_master spidev_slave [software trigger = 1] [calpulser = 0]  [hdfile=headers.dat]  evfile = [events.dat]\n"); 
    return 1; 
  }

  if (nargs > 3) 
  {
    sw_trigger = atoi(args[3]) ; 
  }
  if (nargs > 4) 
  {
    calpulse = atoi(args[4]) ; 
  }

  FILE * fhd = fopen(nargs > 5 ? args[5]: "headers.dat" ,"w"); 
  FILE * fev = fopen(nargs > 6 ? args[6]: "events.dat" ,"w"); 


  signal(SIGINT, catch_interrupt); 
  dev =  nuphase_open(args[1],args[2],0,0); //no interrupt for now and no threadlocking
  
  //enable phased readout 
  nuphase_phased_trigger_readout(dev,1); 

  nuphase_set_readout_number_offset(dev,0); 
//  nuphase_set_buffer_length(dev,127*16); 
  nuphase_calpulse(dev,calpulse); 

  printf("Starting event loop... ctrl-c to cancel!\n"); 

  int nev = 0; 
  struct timespec start; 
  struct timespec now; 
  struct timespec last; 
  struct timespec last_scaler; 

  clock_gettime(CLOCK_REALTIME,&now); 
  clock_gettime(CLOCK_REALTIME,&start); 
  memcpy(&last,&start,sizeof(start)); 
  memcpy(&last_scaler,&start,sizeof(start)); 

  while (!stop)
  {
    int nsent = 0; 
    if (sw_trigger) 
    {
      int itrig = 0;
      nuphase_sw_trigger(dev); 
      nsent++; 
      for (itrig = 0; itrig < (now.tv_nsec / 1000000) % 4 ;itrig++)
      {
        nuphase_sw_trigger(dev); 
        nsent++; 
      }
    }

    int nevents = nuphase_wait_for_and_read_multiple_events(dev,  &hd, &ev); 
    clock_gettime(CLOCK_REALTIME,&now); 
    nev+= nevents; 
    double hz = nev / (now.tv_sec - start.tv_sec + now.tv_nsec *1.e-9 - start.tv_nsec*1.e-9); 
    double instantaneous = nevents  / (now.tv_sec - last.tv_sec + now.tv_nsec *1.e-9 - last.tv_nsec*1.e-9); 
    memcpy(&last,&now, sizeof(last)); 
    printf("Sent %d sw triggers\n", nsent); 
    int missed = (int) (hd[nevents-1].trig_number-hd[nevents-1].event_number); 
    printf("Just read %d events\n\ttotal= %d,  missed=%d (%0.2f%%) ,rate = (%f Hz/%f Hz)\n",nevents,nev, missed,100.*missed/(nev+missed) ,instantaneous, hz); 
    int i; 
    for (i = 0; i < nevents; i++)
    {
      nuphase_event_write(fev,&ev[i]); 
      nuphase_header_write(fhd,&hd[i]); 
    }


    if (now.tv_sec - last_scaler.tv_sec > 10) 
    {
      last_scaler.tv_sec = now.tv_sec; 
      nuphase_status_t status; 
      nuphase_read_status(dev,&status, MASTER); 
      nuphase_status_print(stdout,&status); 
    }

  }
  nuphase_calpulse(dev,0); 
  nuphase_close(dev); 

  fclose(fhd); 
  fclose(fev); 

  return 0; 

}


