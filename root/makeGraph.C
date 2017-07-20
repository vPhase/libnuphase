#include "../nuphase.h" 

TGraph *  makeGraph(const nuphase_event_t * ev, int channel, TGraph * g = 0)
{
  if (!g) g = new TGraph(ev->buffer_length); 

  for (int i = 0; i < ev->buffer_length; i++)
  {
    g->SetPoint(i, i/1.5, ev->data[channel][i]); 
  }

  return g; 

}

