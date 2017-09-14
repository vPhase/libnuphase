#include "nuphasedaq.h" 

#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>
#include <string.h>

// attenuation_scan prefix [n=100] [steps=16] 


#ifdef __arm__ 

uint8_t reverse_bits(uint8_t in) 
{
  uint32_t input = in;
  uint32_t output;
  __asm__("rbit %0, %1\n" : "=r"(output) : "r"(input));
  return output >> 24;
}
#else
uint8_t reverse_bits(uint8_t in) 
{
  uint8_t out = 0; 
  int i; 
  for (i = 0; i < 8; i++) 
  {
    if (in & (1 << i))
    {
      out = out | ( 1 << (7-i)); 
    }
  }

  return out; 
}
#endif


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


  int n = 100; 
  int steps = 16; 
  int step; 
  if (nargs < 2) 
  {
    printf("Usage: attenuation_scan prefix [n=100] [steps=16]\n"); 
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

  signal(SIGINT, catch_interrupt); 
  nuphase_config_t  cmaster; 
  nuphase_config_t  cslave; 
  nuphase_config_init(&cmaster, MASTER); 
  nuphase_config_init(&cslave, SLAVE); 

  cmaster.phased_trigger_readout=0; 

  dev =  nuphase_open("/dev/spidev2.0","/dev/spidev1.0",0,&cmaster,&cslave,0); //no interrupt for now and no threadlocking

  for (step = 0; step < steps; step++) 
  {
    if (stop) break; 
    int attenuation = step * (255/(steps-1)); 
    char header_file_name[512]; 
    char event_file_name[512]; 
    int nevents, ichan; 
    FILE * fhd, *fev;

    printf("Taking %d events at attenuation %d\n", n, attenuation); 
    sprintf(header_file_name,"%s_hd_atten%d.dat", prefix, attenuation); 
    sprintf(event_file_name,"%s_ev_atten%d.dat", prefix, attenuation); 

    fhd  = fopen(header_file_name ,"w"); 
    fev = fopen(event_file_name ,"w"); 

    nuphase_set_readout_number_offset(dev,0); 

    for (ichan = 0; ichan < NP_NUM_CHAN; ichan++) 
    {
      uint8_t reversed = reverse_bits(attenuation); 
      cmaster.attenuation[ichan] = reversed; 
      cslave.attenuation[ichan] = reversed; 
    }

    nuphase_configure(dev, &cmaster, 0, MASTER); 
    nuphase_configure(dev, &cslave, 0, SLAVE); 
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


  nuphase_close(dev); 

  return 0; 

}


