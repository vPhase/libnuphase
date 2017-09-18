#include "nuphasedaq.h" 

#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>
#include <string.h>

// attenuation_scan prefix [n=100] [steps=16] [calpulse=0] 



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


  int calpulse = 0; 
  int n = 100; 
  int steps = 16; 
  int step; 
  if (nargs < 2) 
  {
    printf("Usage: attenuation_scan prefix [n=100] [steps=16] [calpulse=0]\n"); 
    return 1; 
  }

  char * prefix = args[1]; 

  if (nargs > 2) 
  {
    n = atoi(args[2]); 
  }
  if (nargs > 3) 
  {
    steps = atoi(args[3]); 

  }

  if (nargs > 4) 
  {
    calpulse = atoi(args[4]); 
  }


  signal(SIGINT, catch_interrupt); 

  dev =  nuphase_open("/dev/spidev2.0","/dev/spidev1.0",0,0); //no interrupt for now and no threadlocking
  nuphase_calpulse(dev,calpulse); 

  uint8_t master_attenuation[NP_NUM_CHAN]; 
  uint8_t slave_attenuation[NP_NUM_CHAN]; 
  for (step = 0; step < steps; step++) 
  {
    if (stop) break; 
    int attenuation = step * (255/(steps-1)); 
    char header_file_name[512]; 
    char event_file_name[512]; 
    int nevents, ichan; 
    FILE * fhd, *fev;

    printf("Taking %d events at attenuation %d\n", n, attenuation); 
    sprintf(header_file_name,"%shd_atten_%03d.dat", prefix, attenuation); 
    sprintf(event_file_name,"%sev_atten_%03d.dat", prefix, attenuation); 

    fhd  = fopen(header_file_name ,"w"); 
    fev = fopen(event_file_name ,"w"); 

    nuphase_set_readout_number_offset(dev,0); 

    for (ichan = 0; ichan < NP_NUM_CHAN; ichan++) 
    {
      master_attenuation[ichan] = attenuation; 
      slave_attenuation[ichan] = attenuation; 
    }

    nuphase_set_attenuation(dev, master_attenuation, slave_attenuation); 
    nevents = 0;

    while (nevents < n) 
    {
      if (stop) break; 
      nuphase_sw_trigger(dev); 
      nuphase_sw_trigger(dev); 
      nuphase_sw_trigger(dev); 
      nuphase_sw_trigger(dev); 
      int got = nuphase_wait_for_and_read_multiple_events(dev,  &hd, &ev); 

      nevents+= got; 
      int i; 
      for (i = 0; i < got; i++)
      {
        nuphase_event_write(fev,&ev[i]); 
        nuphase_header_write(fhd,&hd[i]); 
      }
    }
    fclose(fhd); 
    fclose(fev); 
  }


  nuphase_calpulse(dev,0); 
  nuphase_close(dev); 

  return 0; 

}


