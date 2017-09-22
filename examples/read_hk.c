#include "nuphasehk.h" 
#include <stdio.h> 
#include <stdlib.h> 

int main(int nargs, char ** args) 
{
  int n = 1; 
  if (nargs > 1) n = atoi(args[1]); 

  nuphase_hk_init(0); 


  nuphase_hk_t hk; 

  int i = 0;
  for (i = 0; i < n; i++) 
  {
    if (i > 0)  
      sleep(1); 
    nuphase_hk(&hk, NP_ASPS_SERIAL); 
    printf("From SERIAL: \n"); 
    nuphase_hk_print(stdout,&hk); 
    nuphase_hk(&hk, NP_ASPS_HTTP); 
    printf("From HTTP: \n"); 
    nuphase_hk_print(stdout,&hk); 
  }




  return 0; 


}
