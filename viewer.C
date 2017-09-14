#include "nuphase.h" 

void viewer(const char * hdfile, const char * evfile, int i = 0) 
{

  FILE * hdf = fopen(hdfile,"r"); 
  FILE * evf = fopen(evfile,"r"); 

  nuphase_event_t ev; 
  nuphase_header_t hd; 

  for (int c = 0; c <= i; c++)
  {
    nuphase_event_read(evf, &ev); 
    nuphase_header_read(hdf, &hd); 
  }

  double * x = new double[ev.buffer_length];
  double * y = new double[ev.buffer_length];

  for (int t = 0; t < ev.buffer_length; t++) 
  {
    x[t] = t * 1./1.5; 
  }

  nuphase_header_print(stdout, &hd); 
  
  for (int ibd = 0; ibd < 2; ibd++)
  {
    if (ev.board_id[ibd])
    {

      TString name = TString::Format("c_%llu_%d", ev.event_number, ev.board_id[ibd]);
      TString title = TString::Format("ev %llu bd%d", ev.event_number, ev.board_id[ibd]);
      TCanvas *c = new TCanvas(name,title, 1800,1000); 
      c->Divide(2,4); 

      for (int ich = 0; ich < NP_NUM_CHAN; ich++)
      {
        c->cd(ich+1); 
        if (hd.channel_read_mask[ibd] & (1 << ich))
        {
          for (int t = 0; t < ev.buffer_length; t++)
          {
            y[t] = ev.data[ibd][ich][t]; 
          }

          TGraph * g = new TGraph(ev.buffer_length, x, y); 
          g->GetXaxis()->SetTitle("t (ns)"); 
          g->GetYaxis()->SetTitle("adu"); 
          g->SetTitle(TString::Format("ch %d\n", ich)); 
          g->Draw("alp"); 
          g->SetEditable(false); 
        }
      }
    }
  }

  delete[] x; 
  delete[] y; 
}
