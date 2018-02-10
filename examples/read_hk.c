#include "nuphasehk.h" 
#include <stdio.h> 
#include <stdlib.h> 
#include <time.h>

int main(int nargs, char ** args) 
{
  int n = 1; 
  if (nargs > 1) n = atoi(args[1]); 

  nuphase_hk_init(0); 

  struct timespec before; 
  struct timespec after; 


  nuphase_hk_t hk; 

  int i = 0;
  for (i = 0; i < n; i++) 
  {
    if (i > 0)  
    {
      sleep(1); 
    }

//    clock_gettime(CLOCK_MONOTONIC, &before); 
//    nuphase_hk(&hk, NP_ASPS_SERIAL); 
 //   clock_gettime(CLOCK_MONOTONIC, &after); 
//    printf("From SERIAL: (dt = %g) \n",  after.tv_sec - before.tv_sec + (1e-9) * ( after.tv_nsec - before.tv_nsec)); 
 //   nuphase_hk_print(stdout,&hk); 
    clock_gettime(CLOCK_MONOTONIC, &before); 
    nuphase_hk(&hk, NP_ASPS_HTTP); 
    clock_gettime(CLOCK_MONOTONIC, &after); 
    printf("From HTTP: (dt = %g) \n",  after.tv_sec - before.tv_sec + (1e-9) * ( after.tv_nsec - before.tv_nsec)); 
    nuphase_hk_print(stdout,&hk); 
  }




  return 0; 


}
