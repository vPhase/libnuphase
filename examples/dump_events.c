#include "nuphase.h" 
#include <stdio.h> 


int main(int nargs, char ** args) 
{

  if (nargs < 2) 
  {
    fprintf(stderr,"dump_events events.dat"); 
    return 1; 
  }



  FILE * f = fopen(args[1], "r"); 
  nuphase_event_t ev;

  while (!nuphase_event_read(f,&ev))
  {
      nuphase_event_print(stdout, &ev, ','); 
  }

  fclose(f); 

  return 0; 





}
