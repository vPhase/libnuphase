#include "nuphasehk.h" 
#include <stdio.h> 
#include <stdlib.h>

//turn_on_boards [http = 0] 
int main(int nargs, char ** args) 
{
  nuphase_hk_t hk; 
  nuphase_asps_method_t method= nargs < 2 || !atoi(args[1]) ? NP_ASPS_SERIAL : NP_ASPS_HTTP; 

  nuphase_hk(&hk, method); 
  printf("Before Turn off: \n"); 
  nuphase_hk_print(stdout,&hk); 

  nuphase_set_gpio_power_state(0,GPIO_FPGA_ALL); 

  printf("after gpio change: \n"); 
  sleep(1); 
  nuphase_hk(&hk, method); 
  nuphase_hk_print(stdout,&hk); 



  printf("%x\n", hk.on_state); 
  nuphase_asps_power_state_t asps =  hk.on_state & ~(NP_POWER_MASTER | NP_POWER_SLAVE); 
  printf("%x\n", asps); 
  nuphase_set_asps_power_state(asps, method); 
  sleep(1); 
  printf("After turn off: \n"); 
  nuphase_hk(&hk, method); 
  nuphase_hk_print(stdout,&hk); 

  return 0; 


}
