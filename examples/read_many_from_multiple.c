#include "nuphasedaq.h" 

#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>
#include <pthread.h> 

// read_many_from_multiple spidev [spidev2] 

volatile static int stop = 0; 

static int ndevices = 0; 
static nuphase_dev_t* devices[128] = {0}; 


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





void * read_device(void * args )
{

  nuphase_header_t hd[4]; 
  nuphase_event_t ev[4]; 

  nuphase_dev_t *dev =  (nuphase_dev_t*) args; 
  int bid = nuphase_get_board_id(dev); 

  char buf[128]; 
  sprintf(buf,"bd%d_headers.dat",bid); 
  FILE * fhd = fopen( buf ,"w"); 
  sprintf(buf,"bd%d_events.dat", bid); 
  FILE * fev = fopen( buf ,"w"); 



  nuphase_set_readout_number_offset(dev,0); 

  printf("Starting event loop on board %d... ctrl-c to cancel!\n",bid); 

  int nev = 0; 
  struct timespec start; 
  struct timespec now; 

  clock_gettime(CLOCK_REALTIME,&start); 
  clock_gettime(CLOCK_REALTIME,&now); 

  while (!stop)
  {
    int ntrigs = 1 + (now.tv_nsec / 10000 ) % 4; 
    int i; 

    for (i = 0;i < ntrigs; i++)
    {
        if (bid == 1) //only do this on first board
        {
          nuphase_sw_trigger(0); // 0 
        }
    }

    int nevents = nuphase_wait_for_and_read_multiple_events(dev,  &hd, &ev); 
    clock_gettime(CLOCK_REALTIME,&now); 
    nev+= nevents; 
    double hz = nev / (now.tv_sec - start.tv_sec + now.tv_nsec *1.e-9 - start.tv_nsec*1.e-9); 
    printf("BD%d: Just read %d events (sent %d, total= %d, avg rate = %f Hz)\n",bid, nevents, ntrigs, nev, hz); 

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

  pthread_t threads[nargs-1]; 
  int i; 
  for (i = 1; i < nargs; i++)
  {
     devices[ndevices] = nuphase_open(args[i], 0,0,1); 
     pthread_create(&threads[i-1],0,read_device,devices[ndevices]); 
     ndevices++; 
  }

  for (i = 0; i < nargs-1; i++)
  {
    pthread_join(threads[i],0);
    printf("Done with thread %d\n",i); 
  }

  return 0; 
}


