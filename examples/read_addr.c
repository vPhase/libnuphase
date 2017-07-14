#include "nuphasedaq.h" 

#include <stdio.h>
#include <sys/time.h> 
#include <stdlib.h>

// read_addr spidev, buffer, start_addr, end_addr, 
int main(int nargs, char ** args )
{

  uint8_t start_addr; 
  uint8_t end_addr; 
  int ichan; 
  int size; 
  int buffer;
  uint8_t *buf[NP_NUM_CHAN]; 
  struct timespec t0; 
  struct timespec t1; 

  nuphase_dev_t * dev;
  nuphase_fwinfo_t fwinfo; 

  if (nargs < 5) 
  {
    printf("Usage: read_addr spidev buffer start_addr end_addr\n"); 
    return 1; 
  }

  buffer = atoi(args[2]); 
  start_addr = atoi(args[3]); 
  end_addr = atoi(args[4]); 
  size = (end_addr-start_addr+1) * NP_NUM_CHUNK * NP_WORD_SIZE; 

  for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
  {
    buf[ichan] = malloc(size); 
  }


  dev =  nuphase_open(args[1],0,0,0); //no interrupt for now and no threadlocking
  nuphase_fwinfo(dev, &fwinfo); 
  nuphase_sw_trigger(dev); 

  clock_gettime(CLOCK_MONOTONIC,&t0); 
  for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
  {
    nuphase_read_raw(dev, buffer, ichan, start_addr, end_addr, buf[ichan]); 
  }

  nuphase_clear_buffer(dev,1 << buffer); 
  clock_gettime(CLOCK_MONOTONIC,&t1); 

  nuphase_close(dev); 

  printf("FIRMWARE fwinfo: %u\n", fwinfo.ver); 
  printf("FIRMWARE DATE: %u\n", fwinfo.date); 
  printf("DNA: %lx\n", fwinfo.dna); 
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


