#include "nuphase.h" 
#include <string.h>
#include <math.h> 


// io (file.dat)
int main(int nargs, const char ** args) 
{

  const char * fname = "file.dat";
  if (nargs > 1) fname = args[1]; 

  nuphase_header_t hd; 
  nuphase_event_t ev; 
  memset(&ev,0,sizeof(ev)); 
  memset(&hd,0,sizeof(hd)); 

  //fill a nonsense header 

  hd.event_number = 12345; 
  hd.trig_number = 23456; 
  hd.buffer_length = 600; 
  hd.pretrigger_samples = 64; 
  hd.readout_time =  1500439356; 
  hd.readout_time_ns =  10005; 
  hd.trig_time =  31415; 
  hd.approx_trigger_time =  1500439355; 
  hd.approx_trigger_time_nsecs =  987654321; 
  hd.triggered_beams = 4; 
  hd.beam_mask = 0x7fff; 

  int i; 
  for (i = 0; i < NP_NUM_BEAMS; i++) 
  {
    hd.beam_power[i] = i*i; 
  }

  hd.deadtime = 0; 
  hd.buffer_number = 0; 
  hd.channel_mask = 0xf; 
  hd.channel_overflow = 0; 
  hd.buffer_mask =3; 
  hd.board_id = 1; 
  hd.trig_type = 2; 
  hd.calpulser = 0; 


//  nuphase_print_header(stdout, &hd); 


  //fill a nonsene event
  ev.event_number = hd.event_number; 
  ev.buffer_length = hd.buffer_length; 
  ev.board_id = hd.board_id; 
  for (i = 0; i < NP_NUM_CHAN; i++)
  {
    int j; 
    for (j = 0; j < hd.buffer_length; j++)
    {
      //some nonsense; 
      ev.data[i][j] = 64 + i * sin( i*j) - i * cos(10*i-100) + i * i * cos(i*j-j*j); 
    }
  }

//  nuphase_print_event(stdout,&ev,'\t'); 


  FILE * f = fopen(fname,"w"); 
  printf("nuphase_header_write returned: %x\n", nuphase_header_write(f, &hd)); 
  printf("nuphase_event_write returned: %x\n", nuphase_event_write(f, &ev)); 
  fclose(f); 

  nuphase_header_t hd2; 
  nuphase_event_t ev2; 
  f = fopen(fname,"r"); 
  printf("nuphase_header_read returned: %x\n", nuphase_header_read(f, &hd2)); 
  printf("nuphase_event_read returned: %x\n", nuphase_event_read(f, &ev2)); 


  fclose(f); 


  printf("memcmp(hd,hd2) returned: %d\n", memcmp(&hd, &hd2, sizeof(hd))); 
  printf("memcmp(ev,ev2) returned: %d\n", memcmp(&ev, &ev2, sizeof(ev))); 

  nuphase_print_header(stdout, &hd2); 
  nuphase_print_event(stdout, &ev,'\t'); 
  nuphase_print_event(stdout, &ev2,'\t'); 


  return 0; 



}
