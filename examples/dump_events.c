#include "nuphase.h" 
#include <stdio.h> 
#include "zlib.h"
#include <string.h> 


int main(int nargs, char ** args) 
{

  if (nargs < 2) 
  {
    fprintf(stderr,"dump_events events.dat"); 
    return 1; 
  }


  int is_zipped = strstr(args[1],".gz") != 0; 

  if (!is_zipped)
  {
    FILE * f = fopen(args[1], "r"); 
    nuphase_event_t ev;

    while (!nuphase_event_read(f,&ev))
    {
        nuphase_event_print(stdout, &ev, ','); 
    }

    fclose(f); 
  }
  else
  {
    gzFile f = gzopen(args[1], "r"); 
    nuphase_event_t ev;

    while (!nuphase_event_gzread(f,&ev))
    {
        nuphase_event_print(stdout, &ev, ','); 
    }

    gzclose(f); 
  }


  return 0; 





}
