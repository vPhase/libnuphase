#include "nuphase.h" 
#include <stdio.h> 
#include <math.h>
#include <stdlib.h>
#include <string.h>


int main(int nargs, char ** args) 
{

  double minsigma= 3; 
  double mindifference= 3; 

  if (nargs < 2) 
  {
    fprintf(stderr,"check_for_glitches events.dat [minsigma = 3] [mindifference = 3]"); 
    return 1; 
  }


  if (nargs > 2) 
  {
    minsigma = atof(args[2]); 
  }

  if (nargs > 3) 
  {
    mindifference = atof(args[3]); 
  }

  FILE * f = fopen(args[1], "r"); 
  nuphase_event_t ev;
  nuphase_event_t last_evs[4]; 

  int ibuf =0;
  for (ibuf = 0; ibuf < 4; ibuf++)
  {
  memset(&last_evs[ibuf],0,sizeof(last_evs[ibuf])); 
  }
  int iev = 0; 
  while (!nuphase_event_read(f,&ev))
  {
    int ichan, isamp, ibd; 
    for (ibd = 0; ibd < NP_MAX_BOARDS; ibd++)
    {
      if (!ev.board_id[ibd]) continue; 

      for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
      {
        double mean = 0, mean2 = 0; 
        for (isamp = 0; isamp < ev.buffer_length; isamp++)
        {
          mean += ev.data[ibd][ichan][isamp]; 
          mean2 += ev.data[ibd][ichan][isamp] * ev.data[ibd][ichan][isamp]; 
        }

        mean /= ev.buffer_length; 
        double rms = sqrt(mean2/ev.buffer_length - mean*mean); 

        for (isamp = 0; isamp < ev.buffer_length; isamp++)
        {
          double diff = fabs(ev.data[ibd][ichan][isamp] - mean); 
          if (diff > minsigma * rms && diff > mindifference) 
          {
            printf("Suspicious value %d\t\tevent index %d\t\t board %d\t\t channel %d\t\t sample %d\t\tchannel mean = %0.3g +/- %0.3g\t\tlast sample: %d \t\tsample at prev evt in buf: %d \n", ev.data[ibd][ichan][isamp], iev, ichan, ev.board_id[ibd], isamp, mean, rms, ev.data[ibd][ichan][isamp-1],last_evs[iev%4].data[ibd][ichan][isamp]); 
          }
        }

      }
    }
    iev++; 
    memcpy(&last_evs[iev % 4], &ev, sizeof(ev)); 
  }

  fclose(f); 

  return 0; 





}
