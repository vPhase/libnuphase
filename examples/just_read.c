#include "nuphasedaq.h" 

#include <stdio.h>
#include <sys/time.h> 
#include <stdlib.h>

// just_read spidev, start_addr, end_addr 
int main(int nargs, char ** args )
{

  uint8_t start_addr; 
  uint8_t end_addr; 
  int ichan; 
  int size; 
  uint8_t *buf[NP_NUM_CHAN]; 
  struct timespec t0; 
  struct timespec t1; 

  nuphase_dev_t * dev;
  nuphase_version_t version; 

  if (nargs < 4) 
  {
    printf("Usage: just_read spidev start_addr end_addr\n"); 
    return 1; 
  }

  start_addr = atoi(args[2]); 
  end_addr = atoi(args[3]); 
  size = (end_addr-start_addr+1) * NP_NUM_CHUNK * NP_WORD_SIZE; 

  for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
  {
    buf[ichan] = malloc(size); 
  }


  dev =  nuphase_open(args[1],0); //no interrupt for now 
  nuphase_version(dev, &version); 
  nuphase_sw_trigger(dev,1); 

  clock_gettime(CLOCK_MONOTONIC,&t0); 
  for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
  {
    nuphase_read_raw(dev, ichan, start_addr, end_addr, buf[ichan]); 
  }
  clock_gettime(CLOCK_MONOTONIC,&t1); 

  nuphase_sw_trigger(dev,0); 
  nuphase_close(dev); 

  printf("FIRMWARE VERSION: %u\n", version.ver); 
  printf("FIRMWARE DATE: %u\n", version.date); 
  printf("DNA: %lx\n", version.dna); 
  printf("Approx time to read out:  %g ms\n", 1000*t1.tv_sec + 1e-6 * t1.tv_nsec - 1000 * t0.tv_sec - 1e-6  * t0.tv_nsec); 


  for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
  {
    int i; 
    printf("\n  CH%d",ichan); 
    for (i = 0; i < size; i++) 
    {
      printf(" %u", buf[ichan][i]); 
    }
    printf("\n\n"); 
  }


  return 0; 

}


