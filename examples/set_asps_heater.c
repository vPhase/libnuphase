#include <stdio.h> 
#include <nuphasehk.h> 
#include <stdlib.h> 


int main(int nargs, char ** args) 
{
  int use_http = 0; 
  int current = 0; 
  if (nargs < 2) 
  {
    printf("set_heater current [use_http = 0]"); 
    return 0; 
  }

  current = atoi(args[1]); 

  if (nargs > 2) 
  {
    use_http = atoi(args[2]); 
  }


  nuphase_set_asps_heater_current(current, use_http ? NP_ASPS_HTTP : NP_ASPS_SERIAL); 
  sleep(1); 
  printf("After setting, current from %s: %d\n", use_http ? "HTTP" : "SERIAL", nuphase_get_asps_heater_current(use_http ? NP_ASPS_HTTP : NP_ASPS_SERIAL)); 

  return 0; 
}
