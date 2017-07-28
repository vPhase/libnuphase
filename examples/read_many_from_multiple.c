#include "nuphasedaq.h" 

#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h> 
#include <stdlib.h>
#include <pthread.h> 

// read_many_from_multiple spidev [spidev2] 

volatile static int stop = 0; 

nuphase_dev_t * dev;
static void catch_interrupt(int signo)
{

  printf("Caught interrupt...\n"); 
  nuphase_cancel_wait(dev); 
  stop =1; 
}





void * read_device(void * args )
{

  nuphase_header_t hd[4]; 
  nuphase_event_t ev[4]; 

  dev =  nuphase_open((const char*)args,0,0,0); //no interrupt for now and no threadlocking
  int bid = nuphase_get_board_id(dev); 

  char buf[128]; 
  sprintf(buf,"bd%d_headers.dat",bid); 
  FILE * fhd = fopen( buf ,"w"); 
  sprintf(buf,"bd%d_events.dat", bid); 
  FILE * fev = fopen( buf ,"w"); 



  nuphase_set_readout_number_offset(dev,0); 

  printf("Starting event loop... ctrl-c to cancel!\n"); 

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
        nuphase_sw_trigger(dev); 
    }

    int nevents = nuphase_wait_for_and_read_multiple_events(dev,  &hd, &ev); 
    clock_gettime(CLOCK_REALTIME,&now); 
    nev+= nevents; 
    double hz = nev / (now.tv_sec - start.tv_sec + now.tv_nsec *1.e-9 - start.tv_nsec*1.e-9); 
    printf("BD%d: Just read %d events (sent %d, total= %d, avg rate = %f Hz)\n",bid, ntrigs, nevents, nev, hz); 

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
     pthread_create(&threads[i-1],0,read_device,args[i]); 
  }

  for (i = 0; i < nargs-1; i++)
  {
    int * thread_ret; 
    pthread_join(threads[i],(void**) &thread_ret);
    printf("Done with thread %d, returned %d\n",i, *thread_ret); 
  }

  return 0; 
}


