#include "nuphasedaq.h" 

#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>

// read_many_from_multiple spidev [spidev2] 

#define MAXDEV 2
volatile static int stop = 0; 

static int ndevices = 0; 
static nuphase_dev_t* devices[MAXDEV] = {0}; 
static FILE * fhd[MAXDEV] = {0}; 
static FILE * fev[MAXDEV] = {0}; 
nuphase_header_t hd[MAXDEV][4]; 
nuphase_event_t ev[MAXDEV][4]; 
int bid[MAXDEV]; 

static void catch_interrupt(int signo)
{
  printf("Caught interrupt...\n"); 

  int i; 
  for (i = 0; i < ndevices; i++)
  {
    nuphase_cancel_wait(devices[i]); 
  }

  stop =1; 
}





int main(int nargs, char ** args) 
{


  if (nargs < 2) 
  {
    printf("Usage: read_many_from_multiple spidev [spidev2]\n"); 
    return 1; 
  }

  struct sigaction saction; 
  sigset_t sset; 
  sigemptyset(&sset); 
  saction.sa_mask = sset; 
  saction.sa_flags = 0; 
  saction.sa_handler = catch_interrupt; 
  sigaction(SIGINT, &saction,0); 


  int iarg; 
  ndevices = nargs-1;

  for (iarg = 1; iarg <nargs; iarg++)
  {
    devices[iarg-1] = nuphase_open(args[iarg],0,0,1); //enable locking 

    bid[iarg-1] = nuphase_get_board_id(devices[iarg-1]); 
    char buf[128]; 
    sprintf(buf,"bd%d_headers.dat",bid[iarg-1]); 
    fhd[iarg-1] = fopen( buf ,"w"); 
    sprintf(buf,"bd%d_events.dat", bid[iarg-1]); 
    fev[iarg-1] = fopen( buf ,"w"); 
    nuphase_set_readout_number_offset(devices[iarg-1],0); 
    nuphase_set_toggle_chipselect(devices[iarg-1],0); 
//    nuphase_set_spi_clock(devices[iarg-1],20); 
  }






  int nev[MAXDEV] = {0}; 
  struct timespec start; 
  struct timespec now; 

  clock_gettime(CLOCK_REALTIME,&start); 
  clock_gettime(CLOCK_REALTIME,&now); 

  int ibd; 
  printf("Starting event loop with %d devices... ctrl-c to cancel!\n",ndevices); 
  while (!stop)
  {
    int ntrigs =0;// 1 + (now.tv_nsec / 10000 ) % 1; 
    int i; 


    for (i = 0;i < ntrigs; i++)
    {
      nuphase_sw_trigger(0); // 0 
    }


    for (ibd = 0; ibd < ndevices; ibd++)
    {

      if (stop) break; 
      nuphase_dev_t * dev = devices[ibd]; 
      int nevents = nuphase_wait_for_and_read_multiple_events(dev,  &hd[ibd], &ev[ibd]); 
      clock_gettime(CLOCK_REALTIME,&now); 
      nev[ibd]+= nevents; 
      double hz = nev[ibd] / (now.tv_sec - start.tv_sec + now.tv_nsec *1.e-9 - start.tv_nsec*1.e-9); 
      printf("BD%d: Just read %d events (sent %d, total= %d, avg rate = %f Hz)\n",bid[ibd], nevents, ntrigs, nev[ibd], hz); 

      for (i = 0; i < nevents; i++)
      {
        nuphase_event_write(fev[ibd],&ev[ibd][i]); 
        nuphase_header_write(fhd[ibd],&hd[ibd][i]); 
      }
    }
  }


  for (ibd = 0; ibd < ndevices; ibd++)
  {
    nuphase_close(devices[ibd]); 
    fclose(fhd[ibd]); 
    fclose(fev[ibd]); 
  }

  return 0; 
}


