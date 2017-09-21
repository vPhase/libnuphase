#include <stdio.h> 
#include <nuphasehk.h> 

int main(int nargs, char ** args) 
{

  printf("Current from HTTP: %d\n", nuphase_get_heater_current(NP_ASPS_HTTP)); 
  printf("Current from SERIAL: %d\n", nuphase_get_heater_current(NP_ASPS_SERIAL)); 

  return 0; 
}
